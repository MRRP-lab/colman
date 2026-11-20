"""Shared fixtures: an isolated ROS graph and a speech_to_text node with no models."""

import os
import random
import sys
import time
from pathlib import Path

import pytest

# Keep test traffic off the real robot graph (and away from other test runs).
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
os.environ["ROS_DOMAIN_ID"] = str(random.randint(100, 200))

sys.path.insert(0, str(Path(__file__).parent))


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "model: needs the real Parakeet/Silero models (cached under "
        "~/.cache/colman/asr); skipped when they are not downloaded",
    )


class FakeClock:
    """Replaces the `time` module inside speech_to_text_node."""

    def __init__(self):
        self.now = 1000.0

    def monotonic(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeVAD:
    """Records what reaches the VAD: ('block', value) or ('reset',)."""

    log = []

    def __init__(self, *args, **kwargs):
        pass

    def __call__(self, block):
        FakeVAD.log.append(("block", float(block[0])))
        return None

    def reset_states(self):
        FakeVAD.log.append(("reset",))


class StubStream:
    """Stands in for sounddevice.InputStream."""

    opened_with = None

    def __init__(self, *args, **kwargs):
        StubStream.opened_with = kwargs

    def start(self):
        pass

    def stop(self):
        pass

    def close(self):
        pass


def spin_until(executor, condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if condition():
            return True
    return condition()


@pytest.fixture
def ros():
    import rclpy

    def init(*params):
        args = ["--ros-args", "-r", f"__ns:=/t{os.getpid()}"]
        for p in params:
            args += ["-p", p]
        rclpy.init(args=args)

    yield init
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def stt_factory(monkeypatch, ros):
    """Build a SpeechToTextNode with fake models, VAD, clock and microphone."""
    from colman_audio_input import speech_to_text_node as stt

    clock = FakeClock()
    FakeVAD.log = []
    StubStream.opened_with = None
    monkeypatch.setattr(stt, "time", clock)
    monkeypatch.setattr(stt, "snapshot_download", lambda *a, **k: "/nonexistent")
    monkeypatch.setattr(stt.onnx_asr, "load_model", lambda *a, **k: object())
    monkeypatch.setattr(stt, "load_silero_vad", lambda: object())
    monkeypatch.setattr(stt, "VADIterator", FakeVAD)
    monkeypatch.setattr(stt.sd, "InputStream", StubStream)
    nodes = []

    def start_ros(*params):
        ros("device:=cpu", *params)

    def make(*params):
        import rclpy

        if not rclpy.ok():
            start_ros(*params)
        node = stt.SpeechToTextNode()
        nodes.append(node)
        return node

    make.clock = clock
    make.start_ros = start_ros
    yield make
    for node in nodes:
        node.destroy_node()
