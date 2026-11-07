from colman_motion.scan import (
    FOUND,
    LOOK_FAILED,
    NOT_VISIBLE,
    STOPPED,
    look_for_tag,
    scan_order,
)

from colman_motion import scan

POSES = ["right", "left"]


class FakeArm:
    def __init__(self, fail_on=None):
        self.moves = []
        self.fail_on = fail_on

    def go_to_configured_pose(self, pose, params):
        self.moves.append(pose)
        return pose != self.fail_on


class FakeWorld:
    """A clock that advances 1 s per tag lookup; each tag is seen from some pose."""

    def __init__(self, arm, visible_from=None):
        self.t = 0.0
        self.arm = arm
        self.visible_from = visible_from or {}  # tag frame -> pose it can be seen from
        self.lookups = 0

    def clock(self):
        return self.t

    def get_tag_pose(self, tag_frame):
        self.t += 1.0
        self.lookups += 1
        here = self.arm.moves[-1] if self.arm.moves else None
        return (
            "pose-of-" + tag_frame if self.visible_from.get(tag_frame) == here else None
        )


def run(world, arm, tag_frame="tag_3", **kwargs):
    return look_for_tag(
        arm, world, tag_frame, "fast", poses=POSES, clock=world.clock, **kwargs
    )


def test_scan_order_puts_the_preferred_pose_first():
    assert scan_order(None, POSES) == ["right", "left"]
    assert scan_order("left", POSES) == ["left", "right"]
    assert scan_order("somewhere else", POSES) == ["right", "left"]


def test_default_scan_poses_are_defined_in_poses_py():
    assert scan.SCAN_POSES  # at least one pose to look from
    assert scan.SCAN_POSES[0] == scan_order()[0]


def test_tag_seen_from_the_first_pose_never_visits_the_second():
    arm = FakeArm()
    world = FakeWorld(arm, {"tag_3": "right"})
    result = run(world, arm)
    assert result.status == FOUND and result.pose == "right"
    assert result.tag_pose == "pose-of-tag_3"
    assert arm.moves == ["right"]


def test_tag_only_visible_from_the_second_pose_is_found_there():
    arm = FakeArm()
    world = FakeWorld(arm, {"tag_3": "left"})
    result = run(world, arm)
    assert result.status == FOUND and result.pose == "left"
    assert arm.moves == ["right", "left"]


def test_first_pose_waits_less_than_the_last():
    arm = FakeArm()
    world = FakeWorld(arm)  # the tag is never visible
    result = run(world, arm, first_timeout_s=4.0, last_timeout_s=10.0)
    assert result.status == NOT_VISIBLE
    assert arm.moves == ["right", "left"]
    assert world.lookups == 4 + 10


def test_preferred_pose_is_tried_first():
    arm = FakeArm()
    world = FakeWorld(arm, {"tag_3": "left"})
    result = run(world, arm, prefer="left")
    assert result.status == FOUND and result.pose == "left"
    assert arm.moves == ["left"]


def test_failed_move_stops_the_search():
    arm = FakeArm(fail_on="right")
    world = FakeWorld(arm, {"tag_3": "left"})
    result = run(world, arm)
    assert result.status == LOOK_FAILED and result.pose == "right"
    assert arm.moves == ["right"]


def test_stop_before_moving():
    arm = FakeArm()
    world = FakeWorld(arm, {"tag_3": "right"})
    result = run(world, arm, stopped=lambda: True)
    assert result.status == STOPPED
    assert arm.moves == []


def test_stop_while_waiting():
    arm = FakeArm()
    world = FakeWorld(arm)
    result = run(world, arm, stopped=lambda: world.lookups >= 2)
    assert result.status == STOPPED
    assert arm.moves == ["right"]


def test_callbacks_run_before_each_move_and_wait():
    arm = FakeArm()
    world = FakeWorld(arm, {"tag_3": "left"})
    events = []
    run(
        world,
        arm,
        on_look=lambda pose: events.append(("look", pose)),
        on_detect=lambda pose: events.append(("detect", pose)),
    )
    assert events == [
        ("look", "right"),
        ("detect", "right"),
        ("look", "left"),
        ("detect", "left"),
    ]
