"""The mic must be deaf while the robot talks (plus a tail) and hear afterwards."""

import numpy as np
import pytest
from conftest import FakeVAD, spin_until
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from colman_audio_input import speech_to_text_node as stt

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TAIL_S = 0.6
TAIL_PARAM = f"mute_tail_ms:={int(TAIL_S * 1000)}"


class Feeder:
    """Injects blocks as the sounddevice callback would; reports what the VAD saw."""

    def __init__(self, node):
        self.node = node
        self.n = 0

    def feed(self):
        """Push one block; True if the VAD saw it, False if it was dropped."""
        self.n += 1
        value = float(self.n)
        block = np.full((stt.BLOCK_SAMPLES, 1), value, dtype=np.float32)
        before = len(FakeVAD.log)
        self.node._on_audio(block, stt.BLOCK_SAMPLES, None, None)
        self.node._audio_q.join()
        new = FakeVAD.log[before:]
        if ("block", value) in new:
            return True
        assert new == [("reset",)], new  # dropped audio resets the segmenter
        return False


@pytest.fixture
def rig(stt_factory):
    """STT node plus a stand-in TTS publisher with the real transient-local QoS."""
    helpers = []

    class Rig:
        clock = stt_factory.clock

        def __init__(self, *params, publish_first=None):
            stt_factory.start_ros(TAIL_PARAM, *params)
            self.helper = Node("tts_stand_in")
            helpers.append(self.helper)
            self.pub = self.helper.create_publisher(Bool, "tts_speaking", LATCHED)
            if publish_first is not None:  # latch a value before the STT node exists
                self.pub.publish(Bool(data=publish_first))
            self.node = stt_factory()
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.executor.add_node(self.helper)
            self.feeder = Feeder(self.node)
            self.feed = self.feeder.feed

        def say(self, value):
            """Publish tts_speaking and wait until the node has processed it."""
            seen = self.node._on_speaking_count
            self.pub.publish(Bool(data=value))
            assert spin_until(
                self.executor, lambda: self.node._on_speaking_count > seen
            )

    yield Rig
    for h in helpers:
        h.destroy_node()


@pytest.fixture(autouse=True)
def count_speaking_msgs(monkeypatch):
    """Count tts_speaking callbacks so tests can wait for delivery exactly."""
    orig = stt.SpeechToTextNode._on_speaking

    def counting(self, msg):
        self._on_speaking_count = getattr(self, "_on_speaking_count", 0) + 1
        orig(self, msg)

    stt.SpeechToTextNode._on_speaking_count = 0
    monkeypatch.setattr(stt.SpeechToTextNode, "_on_speaking", counting)


def test_unmuted_by_default(rig):
    assert rig().feed()


def test_dropped_while_speaking_and_tail_then_accepted(rig):
    r = rig()
    r.say(True)
    assert not r.feed()
    r.clock.advance(30)
    assert not r.feed()  # still speaking, well under mute_max_s
    r.say(False)
    assert not r.feed()  # tail starts now
    r.clock.advance(TAIL_S - 0.05)
    assert not r.feed()
    r.clock.advance(0.1)
    assert r.feed()  # tail over
    assert r.feed()


def test_flapping_restarts_mute(rig):
    r = rig()
    r.say(True)
    r.say(False)
    r.clock.advance(TAIL_S / 2)
    assert not r.feed()
    r.say(True)  # next sentence starts inside the tail
    r.clock.advance(TAIL_S)  # past the first tail, but still speaking
    assert not r.feed()
    r.say(False)
    r.clock.advance(TAIL_S / 2)
    assert not r.feed()  # a fresh tail applies from the last False
    r.clock.advance(TAIL_S)
    assert r.feed()


def test_rapid_flapping_ends_unmuted(rig):
    r = rig()
    for value in [True, False] * 5:
        r.say(value)
    r.clock.advance(TAIL_S + 0.01)
    assert r.feed()


def test_mute_max_s_safety_net(rig):
    """If tts_speaking=False is lost, the mic must eventually reopen."""
    r = rig("mute_max_s:=10.0")
    r.say(True)
    r.clock.advance(9.9)
    assert not r.feed()
    r.clock.advance(0.2)
    assert r.feed()


def test_late_join_latched_true_is_muted(rig):
    """TTS was already speaking before the STT node started."""
    r = rig(publish_first=True)
    assert spin_until(r.executor, lambda: r.node._speaking)
    assert not r.feed()
    r.say(False)
    r.clock.advance(TAIL_S + 0.01)
    assert r.feed()


def test_late_join_latched_false_is_open_after_tail(rig):
    r = rig(publish_first=False)
    assert spin_until(r.executor, lambda: r.node._on_speaking_count == 1)
    assert not r.node._speaking
    r.clock.advance(TAIL_S + 0.01)
    assert r.feed()


def test_mute_topic_disabled_never_mutes(rig):
    r = rig("mute_topic:=''")
    r.pub.publish(Bool(data=True))
    r.executor.spin_once(timeout_sec=0.2)
    assert r.node._on_speaking_count == 0
    assert r.feed()
