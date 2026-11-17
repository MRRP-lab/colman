from types import SimpleNamespace

from colman_motion.scan import (
    FOUND,
    NOT_VISIBLE,
    OVERHEAD,
    REFINE_FAILED,
    STOPPED,
    WRIST,
    locate_tag,
    look_for_tag,
    refine_with_wrist,
)


class FakeArm:
    def __init__(self, ok=True):
        self.moves = []
        self.ok = ok

    def go_to_pose(self, x, y, z, qx, qy, qz, qw, params):
        self.moves.append((x, y, z))
        return self.ok


class FakeTags:
    """A clock that advances 1 s per lookup; frames in `seen` return a pose.

    seen maps a frame to the arm-move count from which it is visible (default 0).
    """

    def __init__(self, seen, arm=None):
        self.t = 0.0
        self.seen = seen
        self.arm = arm
        self.looked = []

    def clock(self):
        return self.t

    def get_tag_pose(self, frame):
        self.t += 1.0
        self.looked.append(frame)
        need = self.seen.get(frame)
        if need is None:
            return None
        moves = len(self.arm.moves) if self.arm else 0
        return f"pose-of-{frame}" if moves >= need else None


def pose_at(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


ESTIMATE = pose_at(0.3, 0.1, 0.02)
ESTIMATE_POSE = SimpleNamespace(translation=ESTIMATE)  # as a transform


def look(tags, **kw):
    return look_for_tag(tags, "tag_3", clock=tags.clock, **kw)


# ----------------------------------------------------------- the order of the look
def test_wrist_is_tried_first_and_overhead_is_not_looked_at():
    tags = FakeTags({"tag_3": 0, "ovh_tag_3": 0})
    scan = look(tags)
    assert (scan.status, scan.source) == (FOUND, WRIST)
    assert scan.tag_pose == "pose-of-tag_3"
    assert set(tags.looked) == {"tag_3"}


def test_overhead_is_used_when_the_wrist_does_not_see_the_tag():
    tags = FakeTags({"ovh_tag_3": 0})
    scan = look(tags, wrist_timeout_s=4.0)
    assert (scan.status, scan.source) == (FOUND, OVERHEAD)
    assert scan.tag_pose == "pose-of-ovh_tag_3"
    # the wrist waited, then the overhead was asked once
    assert tags.looked[-1] == "ovh_tag_3"
    assert set(tags.looked[:-1]) == {"tag_3"}
    assert len(tags.looked) >= 5


def test_not_visible_when_neither_camera_sees_it():
    tags = FakeTags({})
    scan = look(tags)
    assert scan.status == NOT_VISIBLE and scan.tag_pose is None
    assert scan.source is None
    assert tags.looked[-1] == "ovh_tag_3"


def test_zero_wrist_timeout_is_one_look_at_each():
    tags = FakeTags({})
    look(tags, wrist_timeout_s=0.0)
    assert tags.looked == ["tag_3", "ovh_tag_3"]


def test_on_detect_runs_before_the_wrist_wait():
    tags = FakeTags({"tag_3": 0})
    events = []
    look(tags, on_detect=lambda: events.append(list(tags.looked)))
    assert events == [[]]


def test_stop_ends_the_look():
    tags = FakeTags({})
    scan = look(tags, stopped=lambda: len(tags.looked) >= 2)
    assert scan.status == STOPPED


def test_stop_before_the_overhead_look():
    tags = FakeTags({"ovh_tag_3": 0})
    scan = look(tags, wrist_timeout_s=0.0, stopped=lambda: len(tags.looked) >= 1)
    assert scan.status == STOPPED
    assert "ovh_tag_3" not in tags.looked


def test_looking_never_moves_the_arm():
    arm = FakeArm()
    tags = FakeTags({"ovh_tag_3": 0}, arm)
    look(tags)
    look(FakeTags({}, arm))
    assert arm.moves == []


# ------------------------------------------------------------------ the refinement
def refine(arm, tags, **kw):
    return refine_with_wrist(
        arm, tags, "tag_3", ESTIMATE_POSE, "fast", clock=tags.clock, **kw
    )


def test_refine_moves_over_the_estimate_and_returns_the_wrist_pose():
    arm = FakeArm()
    tags = FakeTags({"tag_3": 1}, arm)  # visible once the arm has moved
    scan = refine(arm, tags, height_m=0.3)
    assert (scan.status, scan.source) == (FOUND, WRIST)
    assert scan.tag_pose == "pose-of-tag_3"
    assert arm.moves == [(0.3, 0.1, 0.02 + 0.3)]


def test_refine_failing_to_see_the_tag_is_an_error_not_the_estimate():
    arm = FakeArm()
    tags = FakeTags({"ovh_tag_3": 0}, arm)  # only the overhead camera sees it
    scan = refine(arm, tags)
    assert scan.status == REFINE_FAILED
    assert scan.tag_pose is None
    assert "tag_3" in scan.message
    assert "ovh_tag_3" not in tags.looked


def test_refine_failing_to_move_is_an_error():
    arm = FakeArm(ok=False)
    tags = FakeTags({"tag_3": 0}, arm)
    scan = refine(arm, tags)
    assert scan.status == REFINE_FAILED and scan.tag_pose is None
    assert tags.looked == []


def test_refine_stopped_before_moving():
    arm = FakeArm()
    scan = refine(arm, FakeTags({"tag_3": 0}, arm), stopped=lambda: True)
    assert scan.status == STOPPED and arm.moves == []


# ------------------------------------------------------------------- the whole find
def locate(arm, tags, **kw):
    return locate_tag(arm, tags, "tag_3", "fast", clock=tags.clock, **kw)


class OverheadWorld(FakeTags):
    """The overhead camera sees the tag; the wrist one only once the arm is over it."""

    def get_tag_pose(self, frame):
        pose = super().get_tag_pose(frame)
        if frame == "ovh_tag_3" and pose:
            return ESTIMATE_POSE
        return pose


def test_wrist_find_is_used_directly_without_moving():
    arm = FakeArm()
    scan = locate(arm, FakeTags({"tag_3": 0, "ovh_tag_3": 0}, arm))
    assert (scan.status, scan.source) == (FOUND, WRIST)
    assert arm.moves == []


def test_overhead_find_is_refined_by_the_wrist_camera():
    arm = FakeArm()
    tags = OverheadWorld({"ovh_tag_3": 0, "tag_3": 1}, arm)
    scan = locate(arm, tags)
    assert (scan.status, scan.source) == (FOUND, WRIST)
    assert scan.tag_pose == "pose-of-tag_3"  # not the overhead estimate
    assert len(arm.moves) == 1


def test_overhead_find_that_cannot_be_refined_fails():
    arm = FakeArm()
    tags = OverheadWorld({"ovh_tag_3": 0}, arm)
    scan = locate(arm, tags)
    assert scan.status == REFINE_FAILED and scan.tag_pose is None


def test_nowhere_visible_never_moves_the_arm():
    arm = FakeArm()
    scan = locate(arm, FakeTags({}, arm))
    assert scan.status == NOT_VISIBLE
    assert arm.moves == []
