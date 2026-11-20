import math
from types import SimpleNamespace

import pytest
from colman_motion.arm_control import TOOL_YAWS_DEG, ArmControl
from colman_motion.geometry import yawed

EE_DOWN = (1.0, 0.0, 0.0, 0.0)


def rotate(q, v):
    """v turned by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def test_no_turn_leaves_the_orientation_alone():
    assert yawed(EE_DOWN, 0) == pytest.approx(EE_DOWN)


def test_a_turned_tool_still_points_straight_down():
    for degrees in (0, 90, 180, -90):
        axis = rotate(yawed(EE_DOWN, degrees), (0.0, 0.0, 1.0))
        assert axis == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


def test_a_turn_moves_the_side_the_wrist_sticks_out_to():
    side = (0.0, 1.0, 0.0)  # the tool's own y axis, where the nozzle is offset
    start = rotate(EE_DOWN, side)
    turned = rotate(yawed(EE_DOWN, 180), side)
    assert turned == pytest.approx((-start[0], -start[1], start[2]), abs=1e-9)
    quarter = rotate(yawed(EE_DOWN, 90), side)
    assert quarter[0] * start[0] + quarter[1] * start[1] == pytest.approx(0, abs=1e-9)


def test_the_orientation_stays_a_unit_quaternion():
    for degrees in (0, 90, 180, -90, 37):
        assert math.sqrt(sum(c * c for c in yawed(EE_DOWN, degrees))) == pytest.approx(
            1
        )


def test_quarter_turns_only_so_a_placed_cube_stays_square():
    assert all(d % 90 == 0 for d in TOOL_YAWS_DEG)
    assert TOOL_YAWS_DEG[0] == 0  # the tool as asked comes first


class FakeArm:
    """What vertical_pair needs from the arm: IK, a collision check and a logger."""

    pose_at = staticmethod(ArmControl.pose_at)
    vertical_pair = ArmControl.vertical_pair
    vertical_pair_at = ArmControl.vertical_pair_at

    def __init__(self, blocked_degrees=(), no_ik_degrees=()):
        self.blocked = set(blocked_degrees)
        self.no_ik = set(no_ik_degrees)
        self.messages = []
        self.asked = []
        self.log = SimpleNamespace(
            info=self.messages.append, error=self.messages.append
        )

    def get_logger(self):
        return self.log

    def degrees_of(self, pose):
        for degrees in TOOL_YAWS_DEG:
            if yawed(EE_DOWN, degrees) == pytest.approx(
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
                abs=1e-9,
            ):
                return degrees
        raise AssertionError("an orientation that was not asked for")

    def state_from_ik(self, pose, seed=None):
        degrees = self.degrees_of(pose)
        self.asked.append(degrees)
        return None if degrees in self.no_ik else SimpleNamespace(degrees=degrees)

    def state_from_joints(self, joints):
        return SimpleNamespace(degrees=0)

    def is_valid(self, state):
        return state.degrees not in self.blocked


def test_the_tool_as_asked_is_used_when_it_is_clear():
    arm = FakeArm()
    high, low, tool = arm.vertical_pair(0.3, 0.0, 0.035, 0.145, EE_DOWN)
    assert tool == pytest.approx(EE_DOWN) and arm.asked == [0, 0]


def test_a_neighbour_that_blocks_the_tool_turns_it():
    arm = FakeArm(blocked_degrees=[0])
    high, low, tool = arm.vertical_pair(0.3, -0.12, 0.035, 0.145, EE_DOWN)
    assert tool == pytest.approx(yawed(EE_DOWN, 180))
    assert low.degrees == 180 and high.degrees == 180
    assert any("turned 180" in m for m in arm.messages)


def test_it_goes_on_to_a_quarter_turn_when_two_sides_are_blocked():
    arm = FakeArm(blocked_degrees=[0, 180])
    assert arm.vertical_pair(0.3, -0.12, 0.035, 0.145, EE_DOWN)[2] == pytest.approx(
        yawed(EE_DOWN, 90)
    )


def test_a_turn_with_no_solution_is_skipped():
    arm = FakeArm(no_ik_degrees=[0, 180])
    assert arm.vertical_pair(0.3, 0.0, 0.035, 0.145, EE_DOWN)[2] == pytest.approx(
        yawed(EE_DOWN, 90)
    )


def test_when_no_turn_is_clear_there_is_no_pair_and_the_reason_is_logged():
    arm = FakeArm(blocked_degrees=TOOL_YAWS_DEG)
    assert arm.vertical_pair(0.3, 0.0, 0.035, 0.145, EE_DOWN) is None
    assert any("collision" in m for m in arm.messages)
    arm = FakeArm(no_ik_degrees=TOOL_YAWS_DEG)
    assert arm.vertical_pair(0.3, 0.0, 0.035, 0.145, EE_DOWN) is None
    assert any("No IK" in m for m in arm.messages)
