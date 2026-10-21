#!/usr/bin/env python3
"""Feed a WAV file through speech_to_text_node in place of the microphone.

Runs the real VAD and the real Parakeet model, so it needs the model
downloaded (the first run does that) but no audio hardware. Not collected by
pytest on purpose: it is slow and needs a recording.

    python wav_file_check.py hello.wav --expect "hello world" --device cpu

The WAV must be 16-bit PCM at 16 kHz. Only the first channel is used. Exits 0
if every --expect phrase appears in the transcripts, 1 otherwise.
"""

import argparse
import re
import sys
import time
import wave

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from colman_audio_input import speech_to_text_node as stt

TRAILING_SILENCE_S = 2.0  # must exceed the node's silence_ms so the VAD closes the utterance
MAX_QUEUE = 50  # pace feeding so the node's audio queue (500) never drops blocks
TIMEOUT_S = 120.0


class StubStream:
    """Stands in for sounddevice.InputStream; the test drives _on_audio itself."""

    def __init__(self, *args, **kwargs):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def close(self):
        pass


def read_wav(path):
    with wave.open(path, "rb") as wav:
        if wav.getsampwidth() != 2:
            sys.exit(f"{path}: need 16-bit PCM, got {8 * wav.getsampwidth()}-bit")
        if wav.getframerate() != stt.SAMPLE_RATE:
            sys.exit(f"{path}: need {stt.SAMPLE_RATE} Hz, got {wav.getframerate()} Hz")
        channels = wav.getnchannels()
        raw = wav.readframes(wav.getnframes())
    samples = np.frombuffer(raw, dtype="<i2").reshape(-1, channels)[:, 0]
    return samples.astype(np.float32) / 32768.0


def normalize(text):
    return re.sub(r"[^a-z0-9 ]", "", text.lower()).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("wav")
    parser.add_argument("--expect", action="append", default=[],
                        help="phrase that must appear (case/punctuation ignored); repeatable")
    parser.add_argument("--device", default="auto", choices=["auto", "cuda", "cpu"])
    parser.add_argument("--model", default="nemo-parakeet-tdt-0.6b-v2")
    args = parser.parse_args()

    audio = read_wav(args.wav)
    audio = np.concatenate([audio, np.zeros(int(TRAILING_SILENCE_S * stt.SAMPLE_RATE), np.float32)])
    pad = -len(audio) % stt.BLOCK_SAMPLES
    audio = np.pad(audio, (0, pad))

    rclpy.init(args=["--ros-args", "-p", f"device:={args.device}",
                     "-p", f"model_name:={args.model}"])
    stt.sd.InputStream = StubStream  # before the node tries to open a microphone
    node = stt.SpeechToTextNode()

    # Track in-flight transcription so we know when the pipeline is drained.
    busy = {"n": 0}
    real_transcribe = node._transcribe

    def tracked_transcribe(segment):
        busy["n"] += 1
        try:
            return real_transcribe(segment)
        finally:
            busy["n"] -= 1

    node._transcribe = tracked_transcribe

    transcripts = []
    listener = Node("wav_check_listener")
    listener.create_subscription(
        String, node.get_parameter("topic").value, lambda m: transcripts.append(m.data), 10
    )
    executor = SingleThreadedExecutor()
    executor.add_node(listener)

    try:
        start = time.monotonic()
        for i in range(0, len(audio), stt.BLOCK_SAMPLES):
            block = audio[i:i + stt.BLOCK_SAMPLES].reshape(-1, 1)
            node._on_audio(block, stt.BLOCK_SAMPLES, None, None)
            while node._audio_q.qsize() > MAX_QUEUE:
                time.sleep(0.001)
            executor.spin_once(timeout_sec=0)

        while time.monotonic() - start < TIMEOUT_S:
            executor.spin_once(timeout_sec=0.1)
            drained = (node._audio_q.empty() and node._segment_q.empty()
                       and busy["n"] == 0)
            if drained:
                time.sleep(0.3)  # let a segment still in the VAD thread reach the queue
                executor.spin_once(timeout_sec=0.1)
                if node._audio_q.empty() and node._segment_q.empty() and busy["n"] == 0:
                    break
        else:
            print(f"Timed out after {TIMEOUT_S:.0f} s")
    finally:
        node.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()

    print(f"\nReceived {len(transcripts)} transcript(s):")
    for text in transcripts:
        print(f"  {text!r}")

    joined = normalize(" ".join(transcripts))
    failures = [p for p in args.expect if normalize(p) not in joined]
    if not transcripts:
        failures.append("<no transcripts published>")
    for phrase in failures:
        print(f"FAIL: missing {phrase!r}")
    if not failures:
        print("PASS")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
