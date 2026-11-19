"""TagLookup against synthetic TF: stale, missing, duplicate, flickering tags.

Transforms are put straight into the node's tf2 buffer, so no TF publisher or spinning
is needed. Every test runs under a watchdog: a lookup that hangs fails the test instead
of the run.
"""

import signal
import time

import pytest
import rclpy
from colman_motion.tag_lookup import TagLookup
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

from colman_motion import tag_lookup

NS = 10**9
real_sleep = time.sleep


@pytest.fixture(autouse=True)
def watchdog():
    """Fail (not hang) if a test takes more than 10 s."""

    def on_alarm(signum, frame):
        raise TimeoutError("test hung")

    old = signal.signal(signal.SIGALRM, on_alarm)
    signal.alarm(10)
    yield
    signal.alarm(0)
    signal.signal(signal.SIGALRM, old)


@pytest.fixture
def lookup(monkeypatch):
    rclpy.init()
    # lookup()'s default timeout is bound when the function is defined
    monkeypatch.setattr(TagLookup.lookup, "__defaults__", (0.2,))
    monkeypatch.setattr(tag_lookup, "FRESH_WAIT_S", 0.05)
    node = TagLookup()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def now_ns(node):
    return node.get_clock().now().nanoseconds


def put(node, child, xyz=(0.3, 0.1, 0.02), stamp_ns=None, parent="base_link"):
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    ns = now_ns(node) if stamp_ns is None else stamp_ns
    msg.header.stamp = Time(nanoseconds=ns).to_msg()
    msg.transform.translation.x, msg.transform.translation.y = xyz[0], xyz[1]
    msg.transform.translation.z = xyz[2]
    msg.transform.rotation.w = 1.0
    node.buffer.set_transform(msg, "test")


def elapsed(fn, *args, **kw):
    start = time.monotonic()
    result = fn(*args, **kw)
    return result, time.monotonic() - start


# ------------------------------------------------------------------ missing tag
def test_missing_tag_returns_none_within_the_timeout(lookup):
    result, took = elapsed(lookup.get_tag_pose, "tag_9")
    assert result is None
    assert took < 1.0


def test_missing_base_frame_returns_none(lookup):
    put(lookup, "tag_1")
    assert lookup.get_tag_pose("tag_1", "no_such_base") is None
    assert lookup.lookup("tag_1", "no_such_base", timeout_s=0.0) is None


@pytest.mark.parametrize("name", ["", "/tag_1", "tag 1"])
def test_bad_frame_names_do_not_raise(lookup, name):
    assert lookup.get_tag_pose(name) is None
    assert lookup.lookup(name, "base_link", timeout_s=0.0) is None


def test_lookup_timeout_zero_is_immediate(lookup):
    result, took = elapsed(lookup.lookup, "tag_9", "base_link", timeout_s=0.0)
    assert result is None and took < 0.1


# ------------------------------------------------------------------ stale TF
def test_a_tag_published_once_is_not_reported_as_settled(lookup):
    put(lookup, "tag_1")  # one detection, then the detector stops
    assert lookup.get_tag_pose("tag_1") is None


def test_an_old_transform_is_not_settled(lookup):
    put(lookup, "tag_1", stamp_ns=now_ns(lookup) - 5 * NS)
    assert lookup.get_tag_pose("tag_1") is None


def test_lookup_returns_the_latest_transform(lookup):
    put(lookup, "tag_1", xyz=(0.5, 0.0, 0.0))
    assert lookup.lookup("tag_1", "base_link").transform.translation.x == pytest.approx(
        0.5
    )


def test_stale_chain_gives_none_not_an_exception(lookup):
    # base_link -> camera is fresh, camera -> tag is from long ago: the common time is
    # outside what the buffer holds for the first link (extrapolation).
    put(lookup, "camera", stamp_ns=now_ns(lookup))
    put(lookup, "tag_1", parent="camera", stamp_ns=now_ns(lookup) - 8 * NS)
    assert lookup.get_tag_pose("tag_1") is None
    assert lookup.lookup("tag_1", "base_link", timeout_s=0.0) is None


def test_unconnected_frames_give_none(lookup):
    put(lookup, "tag_1", parent="somewhere_else")  # no path to base_link
    assert lookup.get_tag_pose("tag_1") is None


# ------------------------------------------------------------------ settling
def updating_sleep(lookup, frame, xyz_for_update):
    """A time.sleep that publishes a new transform, as the detector would."""

    def sleep(seconds):
        real_sleep(0.001)  # let the clock tick
        put(lookup, frame, xyz=xyz_for_update, stamp_ns=now_ns(lookup) + 1000)

    return sleep


def test_a_tag_that_keeps_updating_is_reported(lookup, monkeypatch):
    put(lookup, "tag_1", xyz=(0.3, 0.1, 0.02), stamp_ns=now_ns(lookup) - NS // 10)
    monkeypatch.setattr(
        tag_lookup.time, "sleep", updating_sleep(lookup, "tag_1", (0.3005, 0.1, 0.02))
    )
    pose = lookup.get_tag_pose("tag_1")
    assert pose is not None
    assert pose.translation.x == pytest.approx(0.3005)


# ------------------------------------------------------------------ flicker
def test_tag_that_disappears_between_looks_gives_none(lookup, monkeypatch):
    put(lookup, "tag_1")
    calls = []

    def real_lookup(*a, **k):
        calls.append(a)
        return None if len(calls) > 1 else real(*a, **k)

    real = lookup.lookup
    monkeypatch.setattr(lookup, "lookup", real_lookup)
    assert lookup.get_tag_pose("tag_1") is None
    assert len(calls) > 1


def test_tag_that_appears_only_for_the_second_look_gives_none(lookup, monkeypatch):
    def sleep(seconds):
        put(lookup, "tag_1")

    monkeypatch.setattr(tag_lookup.time, "sleep", sleep)
    assert lookup.get_tag_pose("tag_1") is None


def test_flickering_tag_never_raises(lookup, monkeypatch):
    toggle = {"n": 0}

    def sleep(seconds):
        toggle["n"] += 1
        # alternate between two positions, as two detectors disagreeing would
        x = 0.30 if toggle["n"] % 2 else 0.36
        put(lookup, "tag_1", xyz=(x, 0.1, 0.02), stamp_ns=now_ns(lookup) + 1000)

    put(lookup, "tag_1", xyz=(0.36, 0.1, 0.02), stamp_ns=now_ns(lookup) - NS // 10)
    monkeypatch.setattr(tag_lookup.time, "sleep", sleep)
    for _ in range(3):
        lookup.get_tag_pose("tag_1")  # reports an update or None, but must not raise


# ------------------------------------------------------------------ duplicates
def test_tag_with_two_parents_uses_the_latest_without_crashing(lookup):
    # A duplicate detector re-parenting the same child: tf2 keeps one parent's data.
    put(lookup, "camera_a", stamp_ns=now_ns(lookup))
    put(lookup, "camera_b", stamp_ns=now_ns(lookup))
    put(lookup, "tag_3", parent="camera_a", xyz=(0.0, 0.0, 0.5))
    put(lookup, "tag_3", parent="camera_b", xyz=(0.0, 0.0, 0.6))
    lookup.lookup(
        "tag_3", "base_link", timeout_s=0.0
    )  # tf2 may call this ambiguous (None), but must not raise
    lookup.get_tag_pose("tag_3")
