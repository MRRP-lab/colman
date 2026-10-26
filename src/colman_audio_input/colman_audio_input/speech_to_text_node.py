#!/usr/bin/env python3
"""ROS 2 node: microphone -> NVIDIA Parakeet TDT 0.6B -> std_msgs/String topic.

Parakeet runs through onnx-asr (ONNX Runtime), so NeMo is not required.

Parakeet is an offline model, so this node captures the microphone itself,
splits it into utterances with Silero VAD, and transcribes each finished
utterance. Each non-empty transcript is published immediately.
"""

import queue
import threading
import time
from pathlib import Path

import numpy as np
import onnx_asr
import onnxruntime as ort
import rclpy
import sounddevice as sd
import torch
from huggingface_hub import snapshot_download
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from silero_vad import VADIterator, load_silero_vad
from std_msgs.msg import Bool, String

SAMPLE_RATE = 16000
BLOCK_SAMPLES = 512  # Silero VAD requires 512-sample chunks at 16 kHz
BLOCK_S = BLOCK_SAMPLES / SAMPLE_RATE
PRE_ROLL_S = 0.3
MIN_UTTERANCE_S = 0.3
MODEL_REPOS = {
    "nemo-parakeet-tdt-0.6b-v2": "istupakov/parakeet-tdt-0.6b-v2-onnx",
    "nemo-parakeet-tdt-0.6b-v3": "istupakov/parakeet-tdt-0.6b-v3-onnx",
}
MODEL_DIR = Path.home() / ".cache" / "colman" / "asr"


class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__("speech_to_text")

        self.declare_parameter("topic", "speech_text")
        self.declare_parameter("language", "en-US")
        self.declare_parameter("model_name", "nemo-parakeet-tdt-0.6b-v2")
        self.declare_parameter("device", "auto")  # auto | cuda | cpu
        # sounddevice index or name; '' = default
        self.declare_parameter("input_device", "")
        self.declare_parameter("silence_ms", 700)
        self.declare_parameter("max_utterance_s", 30.0)
        self.declare_parameter("vad_threshold", 0.5)
        # Bool topic that is True while the robot is speaking; '' disables muting.
        self.declare_parameter("mute_topic", "tts_speaking")
        # Keep the mic muted this long after speech ends (speaker/room tail).
        self.declare_parameter("mute_tail_ms", 600)

        topic = self.get_parameter("topic").value
        model_name = self.get_parameter("model_name").value
        language = self.get_parameter("language").value
        if model_name.endswith("-v2") and not language.lower().startswith("en"):
            self.get_logger().warn(
                f'{model_name} is English only but language is "{language}". '
                "Use nemo-parakeet-tdt-0.6b-v3 for other languages."
            )

        self.pub = self.create_publisher(String, topic, 10)

        # Mute the mic while the robot speaks so it doesn't transcribe itself.
        self._speaking = False
        self._unmute_at = 0.0
        self._mute_tail_s = int(self.get_parameter("mute_tail_ms").value) / 1000.0
        mute_topic = self.get_parameter("mute_topic").value
        if mute_topic:
            # Match the TTS node's transient-local QoS so we learn the current
            # state even if we start mid-utterance.
            mute_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, mute_topic, self._on_speaking, mute_qos)

        self._silence_ms = int(self.get_parameter("silence_ms").value)
        max_utterance_s = self.get_parameter("max_utterance_s").value
        self._max_samples = int(max_utterance_s * SAMPLE_RATE)
        self._vad_threshold = float(self.get_parameter("vad_threshold").value)

        providers = self._pick_providers(self.get_parameter("device").value)
        self.get_logger().info(
            f"Loading {model_name} with {providers[0]} (first run downloads it)..."
        )
        # onnxruntime rejects the symlinked HF cache layout, so keep a real copy.
        model_path = snapshot_download(
            MODEL_REPOS[model_name], local_dir=MODEL_DIR / model_name
        )
        self._asr = onnx_asr.load_model(model_name, model_path, providers=providers)
        self._check_providers(providers[0])

        self._vad_model = load_silero_vad()

        self._stop = threading.Event()
        self._audio_q = queue.Queue(maxsize=500)  # ~16 s of blocks
        self._segment_q = queue.Queue()
        self._threads = [
            threading.Thread(target=self._segment_loop, name="vad", daemon=True),
            threading.Thread(target=self._asr_loop, name="asr", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

        input_device = self.get_parameter("input_device").value
        if input_device == "":
            input_device = None
        elif input_device.isdigit():
            input_device = int(input_device)
        try:
            self._stream = sd.InputStream(
                samplerate=SAMPLE_RATE,
                channels=1,
                dtype="float32",
                blocksize=BLOCK_SAMPLES,
                device=input_device,
                callback=self._on_audio,
            )
            self._stream.start()
        except sd.PortAudioError as exc:
            self._shutdown_threads()
            raise RuntimeError(f"Could not open the microphone: {exc}") from exc

        self.get_logger().info(f'Listening. Publishing recognized text on "{topic}".')

    @staticmethod
    def _pick_providers(requested):
        cuda = "CUDAExecutionProvider"
        available = ort.get_available_providers()
        if requested == "cuda" or (requested == "auto" and cuda in available):
            return [cuda, "CPUExecutionProvider"]
        return ["CPUExecutionProvider"]

    def _check_providers(self, wanted):
        # onnxruntime silently falls back to CPU if the CUDA libs or driver are missing.
        active = self._asr.asr._encoder.get_providers()[0]
        if active == wanted:
            self.get_logger().info(f"Running on {active}.")
        elif self.get_parameter("device").value == "cuda":
            raise RuntimeError(
                "device=cuda was requested but onnxruntime fell back to "
                f"{active}. Check the NVIDIA driver and CUDA 12 / cuDNN 9 libraries."
            )
        else:
            self.get_logger().warn(f"No usable GPU, running on {active} (slower).")

    def _on_speaking(self, msg):
        if msg.data:
            self._speaking = True
        else:
            self._unmute_at = time.monotonic() + self._mute_tail_s
            self._speaking = False
        self.get_logger().debug(f"tts_speaking={msg.data}")

    def _is_muted(self):
        return self._speaking or time.monotonic() < self._unmute_at

    def _on_audio(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f"Audio status: {status}", throttle_duration_sec=5.0)
        if self._is_muted():
            return
        try:
            self._audio_q.put_nowait(indata[:, 0].copy())
        except queue.Full:
            pass  # segmenter fell behind; drop rather than block the audio thread

    def _segment_loop(self):
        """Group audio blocks into utterances and hand them to the ASR thread."""
        vad = VADIterator(
            self._vad_model,
            threshold=self._vad_threshold,
            sampling_rate=SAMPLE_RATE,
            min_silence_duration_ms=self._silence_ms,
        )
        pre_roll = []
        pre_roll_blocks = int(PRE_ROLL_S / BLOCK_S)
        segment = None  # list of blocks while an utterance is in progress
        n_samples = 0

        while not self._stop.is_set():
            try:
                block = self._audio_q.get(timeout=0.2)
            except queue.Empty:
                continue

            if self._is_muted():
                # Discard anything captured around the robot's own speech.
                pre_roll = []
                segment = None
                vad.reset_states()
                continue

            event = vad(torch.from_numpy(block))
            if segment is None:
                pre_roll.append(block)
                del pre_roll[:-pre_roll_blocks]
                if event and "start" in event:
                    segment = list(pre_roll)
                    n_samples = len(segment) * BLOCK_SAMPLES
                continue

            segment.append(block)
            n_samples += BLOCK_SAMPLES
            ended = event is not None and "end" in event
            if ended or n_samples >= self._max_samples:
                self._finish_segment(segment)
                segment = None
                pre_roll = []
                vad.reset_states()

    def _finish_segment(self, blocks):
        audio = np.concatenate(blocks)
        if len(audio) / SAMPLE_RATE >= MIN_UTTERANCE_S:
            self._segment_q.put(audio)

    def _asr_loop(self):
        while not self._stop.is_set():
            try:
                audio = self._segment_q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                text = self._transcribe(audio)
            except Exception as exc:  # keep listening after a bad utterance
                self.get_logger().error(f"Transcription failed: {exc}")
                continue
            if text and self._is_muted():
                # Segment was captured just before/while the robot started talking.
                self.get_logger().debug(f"Dropped while muted: {text}")
                continue
            if text:
                msg = String()
                msg.data = text
                self.pub.publish(msg)
                self.get_logger().info(f"Published: {text}")

    def _transcribe(self, audio):
        return self._asr.recognize(audio, sample_rate=SAMPLE_RATE).strip()

    def _shutdown_threads(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=5.0)

    def destroy_node(self):
        stream = getattr(self, "_stream", None)
        if stream is not None:
            try:
                stream.stop()
                stream.close()
            except Exception as exc:  # shutting down anyway
                self.get_logger().warn(f"Error stopping microphone: {exc}")
        self._shutdown_threads()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = SpeechToTextNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
