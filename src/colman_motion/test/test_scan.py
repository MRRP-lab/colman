from colman_motion.poses import HOME_POSE
from colman_motion.scan import (
    CLEAR_FAILED,
    FOUND,
    NOT_VISIBLE,
    OVERHEAD,
    STOPPED,
    WRIST,
    locate_tag,
    look_for_tag,
)


class FakeArm:
    def __init__(self, ok=True):
        self.moves = []
        self.ok = ok

    def go_to_joint_pose(self, joints, params):
        self.moves.append(("joints", joints))
        return self.ok

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
    scan = look(tags, stopped=lambda: len(tags.looked) >= 1)
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


# ------------------------------------------------------------------- the whole find
def locate(arm, tags, **kw):
    return locate_tag(arm, tags, "tag_3", "fast", clock=tags.clock, **kw)


def test_wrist_find_is_used_directly_without_moving():
    arm = FakeArm()
    scan = locate(arm, FakeTags({"tag_3": 0, "ovh_tag_3": 0}, arm))
    assert (scan.status, scan.source) == (FOUND, WRIST)
    assert arm.moves == []


def test_overhead_find_is_used_as_it_is_without_moving():
    arm = FakeArm()
    scan = locate(arm, FakeTags({"ovh_tag_3": 0}, arm))
    assert (scan.status, scan.source) == (FOUND, OVERHEAD)
    assert scan.tag_pose == "pose-of-ovh_tag_3"
    assert arm.moves == []


def test_nowhere_visible_only_clears_the_view():
    arm = FakeArm()
    scan = locate(arm, FakeTags({}, arm))
    assert scan.status == NOT_VISIBLE
    assert arm.moves == [("joints", HOME_POSE)]


# ---------------------------------------------------- clearing the overhead view
def look_with_arm(arm, tags, **kw):
    return look_for_tag(tags, "tag_3", arm=arm, params="fast", clock=tags.clock, **kw)


def test_arm_is_cleared_only_when_neither_camera_sees_the_tag():
    arm = FakeArm()
    tags = FakeTags({"ovh_tag_3": 1}, arm)
    scan = look_with_arm(arm, tags)
    assert (scan.status, scan.source) == (FOUND, OVERHEAD)
    assert arm.moves == [("joints", HOME_POSE)]


def test_the_clear_pose_is_shoulder_up_forearm_level():
    assert list(HOME_POSE.values()) == [
        0.0,
        -1.5707963267948966,
        0.0,
        -1.5707963267948966,
        0.0,
        0.0,
    ]


def test_arm_stays_put_when_the_wrist_sees_the_tag():
    arm = FakeArm()
    scan = look_with_arm(arm, FakeTags({"tag_3": 0}, arm))
    assert (scan.status, scan.source) == (FOUND, WRIST)
    assert arm.moves == []


def test_arm_stays_put_when_the_overhead_sees_the_tag_at_once():
    arm = FakeArm()
    scan = look_with_arm(arm, FakeTags({"ovh_tag_3": 0}, arm))
    assert (scan.status, scan.source) == (FOUND, OVERHEAD)
    assert arm.moves == []


def test_not_visible_after_the_arm_is_out_of_the_way():
    arm = FakeArm()
    tags = FakeTags({}, arm)
    scan = look_with_arm(arm, tags)
    assert scan.status == NOT_VISIBLE
    assert arm.moves == [("joints", HOME_POSE)]
    # the wrist camera is not asked again after the move
    assert tags.looked.count("tag_3") == 1
    assert tags.looked[-1] == "ovh_tag_3"


def test_failed_clear_move_is_an_error():
    arm = FakeArm(ok=False)
    scan = look_with_arm(arm, FakeTags({}, arm))
    assert scan.status == CLEAR_FAILED and scan.tag_pose is None


def test_stop_during_the_clear_move():
    arm = FakeArm(ok=False)
    stop = {"now": False}
    arm_go = arm.go_to_joint_pose

    def go(joints, params):
        stop["now"] = True
        return arm_go(joints, params)

    arm.go_to_joint_pose = go
    scan = look_with_arm(arm, FakeTags({}, arm), stopped=lambda: stop["now"])
    assert scan.status == STOPPED
