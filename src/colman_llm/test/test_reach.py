"""The reach model matches the robot description, and answers sensibly."""

import math
import re
from pathlib import Path

import pytest
import yaml
from colman_description.workcell import CUBE_SIZE_M, cube_top_z

from colman_llm import reach

SRC = Path(__file__).resolve().parents[2]
JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
LINK_NAMES = ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3")


# ---------------------------------------------- the copies have not drifted
def test_link_offsets_are_the_factory_calibration():
    path = SRC / "colman_description/colman_description/config/ur_config.yaml"
    kinematics = yaml.safe_load(path.read_text())["kinematics"]
    for name, link in zip(LINK_NAMES, reach.LINKS):
        expected = [kinematics[name][k] for k in "x y z roll pitch yaw".split()]
        assert link == pytest.approx(expected, abs=1e-12), name


def test_joint_limits_are_the_moveit_config_limits():
    path = SRC / "colman_moveit_config/config/joint_limits.yaml"
    limits = yaml.safe_load(path.read_text())["joint_limits"]
    for joint, (low, high) in zip(JOINTS, reach.LIMITS):
        assert low == pytest.approx(limits[joint]["min_position"]), joint
        assert high == pytest.approx(limits[joint]["max_position"]), joint


def test_the_tool_tip_is_where_the_xacro_puts_it():
    text = (
        SRC / "colman_description/colman_description/urdf/colman.urdf.xacro"
    ).read_text()
    tip = re.search(
        r'name="vacuum_tip_joint".*?<origin xyz="([^"]+)"', text, re.DOTALL
    ).group(1)
    assert [float(v) for v in tip.split()] == list(reach.TIP[:3])
    assert "'0.02' if '$(arg use_camera)' == 'true'" in text
    assert reach.MOUNT[2] == 0.02


def test_the_offsets_are_the_ones_the_motion_server_uses():
    text = (SRC / "colman_motion/colman_motion/motion_server.py").read_text()

    def constant(name):
        return float(re.search(rf"^{name} = ([0-9.]+)$", text, re.MULTILINE).group(1))

    assert constant("EE_OFFSET_M") == reach.EE_OFFSET_M
    assert constant("APPROACH_OFFSET_M") == reach.APPROACH_OFFSET_M
    assert constant("PLACE_APPROACH_OFFSET_M") == reach.PLACE_APPROACH_OFFSET_M
    assert "DROP_OFFSET_M = CUBE_SIZE_M" in text
    assert reach.DROP_OFFSET_M == CUBE_SIZE_M


def test_the_tool_yaws_and_home_pose_match_the_motion_package():
    arm = (SRC / "colman_motion/colman_motion/arm_control.py").read_text()
    yaws = re.search(r"^TOOL_YAWS_DEG = \(([^)]*)\)", arm, re.MULTILINE).group(1)
    assert tuple(int(v) for v in yaws.split(",")) == reach.TOOL_YAWS_DEG
    poses = (SRC / "colman_motion/colman_motion/poses.py").read_text()
    assert "[0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]" in poses
    assert reach.HOME == (0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0)


# ------------------------------------------------------------- the model
def test_home_pose_puts_the_tip_where_the_arm_really_is():
    import numpy as np

    position, rotation, _, _ = reach.forward(np.array([reach.HOME]))
    # Measured on the real arm model: shoulder up, forearm level, tool pointing +y.
    assert position[0] == pytest.approx([0.001, 0.312, 0.648], abs=0.002)
    assert rotation[0][:, 2] == pytest.approx([0.0, 1.0, 0.0], abs=0.01)


@pytest.mark.parametrize(
    "x, y, level",
    [(0.25, 0.0, 1), (0.35, 0.0, 1), (0.25, 0.2, 1), (0.25, -0.2, 1), (0.3, 0.0, 4)],
)
def test_cubes_in_front_of_the_arm_are_reachable(x, y, level):
    z = cube_top_z(level)
    assert reach.can_pick(x, y, z)
    assert reach.can_place_on(x, y, z)


@pytest.mark.parametrize("x, y", [(0.8, 0.0), (0.0, 0.8), (0.05, 0.0), (0.6, 0.5)])
def test_cubes_too_far_or_too_near_the_base_are_not(x, y):
    z = cube_top_z(1)
    assert not reach.can_pick(x, y, z)
    assert not reach.can_place_on(x, y, z)


def test_a_place_is_stricter_than_a_pick_at_the_edge():
    """Far forward the arm can pick a cube it can only just reach, but won't place."""
    z = cube_top_z(1)
    edge = [x / 100 for x in range(40, 70)]
    picks = [x for x in edge if reach.can_pick(x, 0.0, z)]
    places = [x for x in edge if reach.can_place_on(x, 0.0, z)]
    assert places and picks and max(places) < max(picks)


def test_a_tall_stack_reaches_less_far_than_a_cube_on_the_table():
    def furthest(level):
        xs = [x / 100 for x in range(20, 70)]
        return max(x for x in xs if reach.can_place_on(x, 0.0, cube_top_z(level)))

    assert furthest(4) < furthest(1)
