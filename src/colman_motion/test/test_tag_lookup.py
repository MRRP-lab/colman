"""TagLookup and scan against synthetic TF: stale, missing, duplicate, flickering tags.

Transforms are put straight into the node's tf2 buffer, so no TF publisher or spinning
is needed. Every test runs under a watchdog: a lookup that hangs fails the test instead
of the run.
"""

import signal
import time

import pytest
import rclpy
from colman_motion.tag_lookup import TagLookup, within_tolerance
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

from colman_motion import scan, tag_lookup

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
    monkeypatch.setattr(tag_lookup, "WAIT_S", 0.05)
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


# ------------------------------------------------------------------ within_tolerance
def test_within_tolerance_uses_the_largest_axis():
    class P:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    assert within_tolerance(P(0, 0, 0), P(0.002, 0.002, 0.002), 0.003)
    assert not within_tolerance(P(0, 0, 0), P(0, 0, 0.004), 0.003)


@pytest.mark.parametrize("axis", "xyz")
def test_a_nan_position_is_never_within_tolerance(axis):
    """A failed pose estimate must not read as "the tag held still"."""

    class P:
        x = y = z = 0.0

    bad = P()
    setattr(bad, axis, float("nan"))
    assert not within_tolerance(P(), bad, 0.003)
    assert not within_tolerance(bad, bad, 0.003)


# ------------------------------------------------------------------ missing tag
def test_missing_tag_returns_none_within_the_timeout(lookup):
    result, took = elapsed(lookup.get_tag_pose, "tag_9")
    assert result is None
    assert took < 1.0


def test_missing_base_frame_returns_none(lookup):
    put(lookup, "tag_1")
    assert lookup.get_tag_pose("tag_1", "no_such_base") is None
    assert lookup.peek("tag_1", "no_such_base") is None


@pytest.mark.parametrize("name", ["", "/tag_1", "tag 1"])
def test_bad_frame_names_do_not_raise(lookup, name):
    assert lookup.get_tag_pose(name) is None
    assert lookup.peek(name) is None


def test_peek_does_not_wait(lookup):
    result, took = elapsed(lookup.peek, "tag_9")
    assert result is None and took < 0.1


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


def test_peek_returns_the_latest_transform(lookup):
    put(lookup, "tag_1", xyz=(0.5, 0.0, 0.0))
    assert lookup.peek("tag_1").translation.x == pytest.approx(0.5)


def test_stale_chain_gives_none_not_an_exception(lookup):
    # base_link -> camera is fresh, camera -> tag is from long ago: the common time is
    # outside what the buffer holds for the first link (extrapolation).
    put(lookup, "camera", stamp_ns=now_ns(lookup))
    put(lookup, "tag_1", parent="camera", stamp_ns=now_ns(lookup) - 8 * NS)
    assert lookup.get_tag_pose("tag_1") is None
    assert lookup.peek("tag_1") is None


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


def test_a_steady_tag_is_reported(lookup, monkeypatch):
    put(lookup, "tag_1", xyz=(0.3, 0.1, 0.02), stamp_ns=now_ns(lookup) - NS // 10)
    monkeypatch.setattr(
        tag_lookup.time, "sleep", updating_sleep(lookup, "tag_1", (0.3005, 0.1, 0.02))
    )
    pose = lookup.get_tag_pose("tag_1")
    assert pose is not None
    assert pose.translation.x == pytest.approx(0.3005)


def test_a_moving_tag_is_not_reported(lookup, monkeypatch):
    put(lookup, "tag_1", stamp_ns=now_ns(lookup) - NS // 10)
    monkeypatch.setattr(
        tag_lookup.time, "sleep", updating_sleep(lookup, "tag_1", (0.35, 0.1, 0.02))
    )
    assert lookup.get_tag_pose("tag_1") is None


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
    assert len(calls) == 2


def test_tag_that_appears_only_for_the_second_look_gives_none(lookup, monkeypatch):
    def sleep(seconds):
        put(lookup, "tag_1")

    monkeypatch.setattr(tag_lookup.time, "sleep", sleep)
    assert lookup.get_tag_pose("tag_1") is None


def test_flickering_tag_never_settles_but_never_raises(lookup, monkeypatch):
    toggle = {"n": 0}

    def sleep(seconds):
        toggle["n"] += 1
        # alternate between two positions, as two detectors disagreeing would
        x = 0.30 if toggle["n"] % 2 else 0.36
        put(lookup, "tag_1", xyz=(x, 0.1, 0.02), stamp_ns=now_ns(lookup) + 1000)

    put(lookup, "tag_1", xyz=(0.36, 0.1, 0.02), stamp_ns=now_ns(lookup) - NS // 10)
    monkeypatch.setattr(tag_lookup.time, "sleep", sleep)
    for _ in range(3):
        assert lookup.get_tag_pose("tag_1") is None


# ------------------------------------------------------------------ duplicates
def test_wrist_and_overhead_copies_are_distinct_frames(lookup):
    put(lookup, "tag_3", xyz=(0.30, 0.0, 0.0))
    put(lookup, "ovh_tag_3", xyz=(0.31, 0.0, 0.0))
    assert lookup.peek("tag_3").translation.x == pytest.approx(0.30)
    assert lookup.peek(scan.overhead_frame("tag_3")).translation.x == pytest.approx(
        0.31
    )


def test_tag_with_two_parents_uses_the_latest_without_crashing(lookup):
    # A duplicate detector re-parenting the same child: tf2 keeps one parent's data.
    put(lookup, "camera_a", stamp_ns=now_ns(lookup))
    put(lookup, "camera_b", stamp_ns=now_ns(lookup))
    put(lookup, "tag_3", parent="camera_a", xyz=(0.0, 0.0, 0.5))
    put(lookup, "tag_3", parent="camera_b", xyz=(0.0, 0.0, 0.6))
    lookup.peek("tag_3")  # tf2 may call this ambiguous (None), but must not raise
    lookup.get_tag_pose("tag_3")


# ------------------------------------------------------------------ scan with real TF
class RealTags:
    def __init__(self, lookup):
        self.lookup = lookup

    def get_tag_pose(self, frame):
        return self.lookup.get_tag_pose(frame)

    def peek(self, frame):
        return self.lookup.peek(frame)


def test_scan_reports_not_visible_without_hanging(lookup):
    result, took = elapsed(
        scan.look_for_tag,
        RealTags(lookup),
        "tag_3",
        wrist_timeout_s=0.0,
        overhead_timeout_s=0.0,
    )
    assert result.status == scan.NOT_VISIBLE
    assert took < 2.0


def test_scan_wrist_wins_when_both_cameras_see_the_tag(lookup, monkeypatch):
    put(lookup, "tag_3", xyz=(0.30, 0.0, 0.0), stamp_ns=now_ns(lookup) - NS // 10)
    put(lookup, "ovh_tag_3", xyz=(0.34, 0.0, 0.0))
    monkeypatch.setattr(
        tag_lookup.time, "sleep", updating_sleep(lookup, "tag_3", (0.3, 0.0, 0.0))
    )
    result = scan.look_for_tag(RealTags(lookup), "tag_3")
    assert (result.status, result.source) == (scan.FOUND, scan.WRIST)
    assert result.tag_pose.translation.x == pytest.approx(0.30)


def test_scan_falls_back_to_the_overhead_tag(lookup, monkeypatch):
    put(lookup, "ovh_tag_3", xyz=(0.34, 0.0, 0.0), stamp_ns=now_ns(lookup) - NS // 10)
    monkeypatch.setattr(
        tag_lookup.time, "sleep", updating_sleep(lookup, "ovh_tag_3", (0.34, 0, 0))
    )
    result = scan.look_for_tag(RealTags(lookup), "tag_3")
    assert (result.status, result.source) == (scan.FOUND, scan.OVERHEAD)


def test_compare_cameras_ignores_an_absent_overhead_tag(lookup):
    class Pose:
        class translation:
            x = y = z = 0.0

    assert scan.compare_cameras(RealTags(lookup), "tag_3", Pose) is None


# ------------------------------------------------------------------ scan edge cases
class ScriptedTags:
    def __init__(self, answers):
        self.answers = {k: list(v) for k, v in answers.items()}
        self.looked = []

    def get_tag_pose(self, frame):
        self.looked.append(frame)
        queue = self.answers.get(frame, [])
        return queue.pop(0) if queue else None


def test_scan_stop_while_waiting_returns_stopped_quickly():
    tags = ScriptedTags({})
    calls = {"n": 0}

    def stopped():
        calls["n"] += 1
        return calls["n"] > 3

    result, took = elapsed(
        scan.look_for_tag, tags, "tag_3", stopped=stopped, wrist_timeout_s=60.0
    )
    assert result.status == scan.STOPPED
    assert took < 2.0


def test_scan_flicker_in_wrist_view_is_found_when_it_reappears():
    tags = ScriptedTags({"tag_3": [None, None, "pose"]})
    result = scan.look_for_tag(tags, "tag_3", wrist_timeout_s=5.0)
    assert (result.status, result.tag_pose) == (scan.FOUND, "pose")
    assert tags.looked == ["tag_3"] * 3


def test_scan_zero_timeout_looks_exactly_once():
    tags = ScriptedTags({})
    scan.look_for_tag(tags, "tag_3", wrist_timeout_s=0.0, overhead_timeout_s=0.0)
    assert tags.looked == ["tag_3", "ovh_tag_3"]


def test_scan_arm_that_cannot_clear_is_reported():
    class Arm:
        def go_to_joint_pose(self, joints, params):
            return False

    result = scan.look_for_tag(ScriptedTags({}), "tag_3", arm=Arm(), params=None)
    assert result.status == scan.CLEAR_FAILED and result.message


def test_scan_overhead_appears_after_the_arm_clears():
    tags = ScriptedTags({"ovh_tag_3": [None, None, "pose"]})

    class Arm:
        def go_to_joint_pose(self, joints, params):
            return True

    result = scan.look_for_tag(
        tags, "tag_3", arm=Arm(), params=None, overhead_timeout_s=5.0
    )
    assert (result.status, result.source) == (scan.FOUND, scan.OVERHEAD)
