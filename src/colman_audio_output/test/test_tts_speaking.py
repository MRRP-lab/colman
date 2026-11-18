"""kokoro_tts with a fake synthesis backend: tts_speaking must never stick True."""

import os
import random
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
os.environ["ROS_DOMAIN_ID"] = str(random.randint(100, 200))

from colman_audio_output import kokoro_tts  # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FakePipeline:
    """Stands in for kokoro.KPipeline; behaviour is set per test."""

    def __init__(self, *args, **kwargs):
        self.calls = []
        self.gate = None  # threading.Event: block synthesis until set
        self.fail_on = lambda text: False
        self.fail_midway = False

    def __call__(self, text, voice=None, speed=1.0):
        self.calls.append(text)
        if self.gate is not None:
            assert self.gate.wait(10)
        if self.fail_on(text):
            raise RuntimeError("synthesis exploded")
        yield "g", "p", np.zeros(240, dtype=np.float32)
        if self.fail_midway:
            raise RuntimeError("died mid-stream")
        yield "g", "p", None  # Kokoro may yield no audio for a segment
        yield "g", "p", np.zeros(240, dtype=np.float32)


class Rig:
    def __init__(self, node, pipeline, tmp_path):
        self.node, self.pipeline, self.tmp_path = node, pipeline, tmp_path
        self.helper = Node("tts_test_peer")
        self.states = []
        self.helper.create_subscription(
            Bool, "tts_speaking", lambda m: self.states.append(m.data), LATCHED
        )
        self.pub = self.helper.create_publisher(String, "llm_response", 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.helper)
        self.executor.add_node(node)

    def spin(self, seconds=0.3):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)

    def spin_until(self, cond, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)
            if cond():
                return True
        return cond()

    def say(self, text):
        self.pub.publish(String(data=text))

    def wait_idle(self, expected_calls=None):
        """Wait until the worker has drained and published its final state."""

        def idle():
            done = expected_calls is None or len(self.pipeline.calls) >= expected_calls
            return done and self.node._queue.empty() and self.states[-1] is False

        return self.spin_until(idle) and self._settle()

    def _settle(self):
        self.spin(0.2)
        return True

    @property
    def saved(self):
        return sorted(self.tmp_path.glob("msg_*.wav"))


@pytest.fixture
def rig(monkeypatch, tmp_path):
    pipeline = FakePipeline()
    monkeypatch.setattr(kokoro_tts, "KPipeline", lambda *a, **k: pipeline)
    rclpy.init(
        args=[
            "--ros-args",
            "-r",
            f"__ns:=/t{os.getpid()}",
            "-p",
            "mode:=save",
            "-p",
            f"output_dir:={tmp_path}",
        ]
    )
    node = kokoro_tts.KokoroTTSNode()
    r = Rig(node, pipeline, tmp_path)
    assert r.spin_until(lambda: r.states == [False])  # latched initial state
    yield r
    if pipeline.gate:
        pipeline.gate.set()
    node.shutdown()
    r.helper.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def test_initial_state_is_latched_false(rig):
    assert rig.states == [False]


def test_speaking_true_then_false_around_synthesis(rig):
    rig.say("Hello there.")
    assert rig.wait_idle(1)
    assert rig.states == [False, True, False]
    assert len(rig.saved) == 1


def test_true_is_published_before_synthesis_starts(rig):
    rig.pipeline.gate = threading.Event()
    rig.say("Hello there.")
    assert rig.spin_until(lambda: rig.states[-1:] == [True])
    assert rig.spin_until(lambda: rig.pipeline.calls == ["Hello there."])
    assert rig.states == [False, True]  # still synthesizing
    rig.pipeline.gate.set()
    assert rig.wait_idle(1)
    assert rig.states == [False, True, False]


@pytest.mark.parametrize("text", ["", "   ", "\n\t \n"])
def test_blank_text_is_ignored(rig, text):
    rig.say(text)
    rig.spin(0.5)
    assert rig.pipeline.calls == []
    assert rig.states == [False]


def test_text_is_stripped(rig):
    rig.say("  hi  ")
    assert rig.wait_idle(1)
    assert rig.pipeline.calls == ["hi"]


def test_very_long_text(rig):
    text = "This is a sentence. " * 2000  # ~40k characters
    rig.say(text)
    assert rig.wait_idle(1)
    assert rig.pipeline.calls == [text.strip()]
    assert rig.states == [False, True, False]


def test_back_to_back_stay_speaking_without_gap(rig):
    rig.pipeline.gate = threading.Event()
    for i in range(3):
        rig.say(f"message {i}")
    assert rig.spin_until(lambda: len(rig.pipeline.calls) >= 1)
    rig.spin(0.3)  # let all three reach the queue
    rig.pipeline.gate.set()
    assert rig.wait_idle(3)
    assert rig.pipeline.calls == ["message 0", "message 1", "message 2"]
    # One continuous True: the mic must not reopen between messages.
    assert rig.states == [False, True, False]


def test_synthesis_exception_does_not_leave_speaking_true(rig):
    rig.pipeline.fail_on = lambda text: True
    rig.say("this will fail")
    assert rig.wait_idle(1)
    assert rig.states[-1] is False
    assert rig.states == [False, True, False]


def test_exception_midway_through_stream_clears_speaking(rig):
    rig.pipeline.fail_midway = True
    rig.say("partial audio then crash")
    assert rig.wait_idle(1)
    assert rig.states[-1] is False


def test_worker_survives_exception_and_speaks_next_message(rig):
    rig.pipeline.fail_on = lambda text: "bad" in text
    rig.say("bad one")
    assert rig.wait_idle(1)
    rig.say("good one")
    assert rig.wait_idle(2)
    assert rig.states[-1] is False
    assert len(rig.saved) == 1  # only the good one produced audio


def test_exception_with_message_queued_behind_still_ends_false(rig):
    rig.pipeline.gate = threading.Event()
    rig.pipeline.fail_on = lambda text: "bad" in text
    rig.say("bad one")
    assert rig.spin_until(lambda: len(rig.pipeline.calls) == 1)
    rig.say("good one")
    rig.spin(0.3)
    rig.pipeline.gate.set()
    assert rig.wait_idle(2)
    assert rig.states[-1] is False
    assert len(rig.saved) == 1


def test_player_failure_clears_speaking(rig, monkeypatch):
    """E.g. mpv missing or crashing in stream mode."""
    rig.node.mode = "stream"

    def boom(*a, **k):
        raise FileNotFoundError("mpv")

    monkeypatch.setattr(kokoro_tts.subprocess, "Popen", boom)
    rig.say("no player")
    assert rig.wait_idle()
    assert rig.states[-1] is False


def test_late_joiner_sees_current_speaking_state(rig):
    rig.pipeline.gate = threading.Event()
    rig.say("long utterance")
    assert rig.spin_until(lambda: rig.states[-1:] == [True])
    late = []
    sub = rig.helper.create_subscription(
        Bool, "tts_speaking", lambda m: late.append(m.data), LATCHED
    )
    assert rig.spin_until(lambda: late == [True])
    rig.pipeline.gate.set()
    assert rig.wait_idle(1)
    assert late[-1] is False
    rig.helper.destroy_subscription(sub)


def test_shutdown_while_speaking_publishes_false(rig):
    """A subscriber must not stay muted until its safety timeout if we exit."""
    rig.pipeline.gate = threading.Event()
    rig.say("interrupted")
    assert rig.spin_until(lambda: rig.states[-1:] == [True])
    threading.Timer(0.3, rig.pipeline.gate.set).start()
    rig.node.shutdown()
    assert rig.spin_until(lambda: rig.states[-1:] == [False])
