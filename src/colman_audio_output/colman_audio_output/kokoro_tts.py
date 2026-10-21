#!/usr/bin/env python3
"""ROS 2 node: subscribes to LLM text output and speaks it with local Kokoro-82M TTS.

Subscribes:  <input_topic>  (std_msgs/String)  -- text from the LLM node
Publishes:   tts_speaking   (std_msgs/Bool)    -- True while audio is playing
                                                  (handy for muting the mic node)

Run with:  ros2 run colman_audio_output kokoro_tts
Kokoro-82M (Apache 2.0) runs locally; weights are downloaded from Hugging Face on
first use. Needs mpv (stream mode) or ffplay from ffmpeg (play mode)
on the PATH. These come with the pixi env.
"""

import hashlib
import os
import queue
import shutil
import subprocess
import threading

import numpy as np
import rclpy
import soundfile as sf
from kokoro import KPipeline
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

SAMPLE_RATE = 24000  # Kokoro's fixed output rate

# External player each mode shells out to (save mode needs none).
PLAYERS = {"stream": "mpv", "play": "ffplay", "save": None}


class KokoroTTSNode(Node):
    def __init__(self):
        super().__init__("kokoro_tts")

        # ---- Parameters ----
        self.declare_parameter("input_topic", "llm_response")
        self.declare_parameter("voice", "af_heart")  # Kokoro voice pack name
        self.declare_parameter("lang_code", "a")  # 'a' = American, 'b' = British
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("mode", "stream")  # stream | play | save
        self.declare_parameter("output_dir", "/tmp/tts")  # used when mode == save
        self.declare_parameter("max_queue", 5)  # pending messages; 0 = unbounded

        input_topic = self.get_parameter("input_topic").value
        self.voice = self.get_parameter("voice").value
        self.speed = self.get_parameter("speed").value
        self.mode = self.get_parameter("mode").value
        self.output_dir = self.get_parameter("output_dir").value

        if self.mode not in PLAYERS:
            raise RuntimeError(f"Invalid mode '{self.mode}' (use {', '.join(PLAYERS)})")
        player = PLAYERS[self.mode]
        if player and not shutil.which(player):
            raise RuntimeError(f"mode '{self.mode}' needs '{player}' on the PATH")

        # ---- Kokoro pipeline (loads the model; downloads weights on first run) ----
        self.get_logger().info("Loading Kokoro-82M...")
        self.pipeline = KPipeline(
            lang_code=self.get_parameter("lang_code").value,
            repo_id="hexgrad/Kokoro-82M",
        )

        # ---- ROS interfaces ----
        # Transient-local so a mic node that starts mid-utterance still sees
        # the current speaking state.
        speaking_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.speaking_pub = self.create_publisher(Bool, "tts_speaking", speaking_qos)
        self._set_speaking(False)
        self.create_subscription(String, input_topic, self._on_text, 10)

        # ---- Worker thread: TTS calls block, so keep them out of the callback ----
        self._queue: "queue.Queue[str | None]" = queue.Queue(
            maxsize=self.get_parameter("max_queue").value
        )
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"Listening on '{input_topic}' | voice={self.voice} | mode={self.mode}"
        )

    # ------------------------------------------------------------------ helpers
    def _set_speaking(self, value: bool):
        self.speaking_pub.publish(Bool(data=value))

    def _synthesize(self, text: str):
        """Yield float32 mono audio chunks, one per sentence-ish segment."""
        for _, _, audio in self.pipeline(text, voice=self.voice, speed=self.speed):
            if audio is not None:
                yield np.asarray(audio, dtype=np.float32)

    def _player_cmd(self) -> list[str]:
        # Raw 32-bit float PCM on stdin.
        if self.mode == "stream":
            return [
                "mpv", "--no-terminal", "--no-video", "--demuxer=rawaudio",
                "--demuxer-rawaudio-format=floatle",
                f"--demuxer-rawaudio-rate={SAMPLE_RATE}",
                "--demuxer-rawaudio-channels=1", "-",
            ]  # fmt: skip
        return [
            "ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet",
            "-f", "f32le", "-ar", str(SAMPLE_RATE), "-ch_layout", "mono", "-",
        ]  # fmt: skip

    def _play(self, chunks):
        """Pipe audio to the player; stream mode starts playing on the first chunk."""
        if self.mode == "play":
            chunks = [np.concatenate(list(chunks))]  # synthesize everything first
        proc = subprocess.Popen(self._player_cmd(), stdin=subprocess.PIPE)
        try:
            for chunk in chunks:
                proc.stdin.write(chunk.tobytes())
        finally:
            proc.stdin.close()
            proc.wait()

    def _speak(self, text: str):
        chunks = self._synthesize(text)
        if self.mode != "save":
            self._play(chunks)
            return
        os.makedirs(self.output_dir, exist_ok=True)
        name = hashlib.sha1(text.encode("utf-8")).hexdigest()[:16]
        path = os.path.join(self.output_dir, f"msg_{name}.wav")
        sf.write(path, np.concatenate(list(chunks)), SAMPLE_RATE)
        self.get_logger().info(f"Saved audio to {path}")

    # ---------------------------------------------------------------- callbacks
    def _on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        try:
            self._queue.put_nowait(text)
        except queue.Full:
            # Keep the newest message: drop the oldest pending one instead.
            try:
                dropped = self._queue.get_nowait()
            except queue.Empty:
                dropped = None
            self.get_logger().warn(f"TTS queue full; dropping: {str(dropped)[:80]}")
            try:
                self._queue.put_nowait(text)
            except queue.Full:
                self.get_logger().warn(f"TTS queue full; dropping: {text[:80]}")

    def _worker_loop(self):
        while True:
            text = self._queue.get()
            if text is None:
                return
            preview = text[:80] + ("..." if len(text) > 80 else "")
            self.get_logger().info(f"Speaking: {preview}")
            self._set_speaking(True)
            try:
                self._speak(text)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"TTS failed: {e}")
            finally:
                # Stay "speaking" across back-to-back messages so a muted mic
                # doesn't reopen between them.
                if self._queue.empty():
                    self._set_speaking(False)

    def shutdown(self):
        # Drop pending messages so the sentinel isn't stuck behind them.
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass  # worker is a daemon thread; it exits with the process
        self._worker.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KokoroTTSNode()
        rclpy.spin(node)
    except RuntimeError as e:
        rclpy.logging.get_logger("kokoro_tts").fatal(str(e))
        raise SystemExit(1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
