from functools import partial

import numpy as np
from colman_motion.follow_geometry import (
    ABOVE_CUBE_M,
    MAX_ANGULAR_RPS,
    MAX_LINEAR_MPS,
    MAX_REACH_M,
    SHOULDER_POSITION,
    STANDOFF_M,
    axis_from_fourfold,
    chase_position,
    clamp_norm,
    fourfold,
    horizontal_axis,
    level_rotation,
    lowpass,
)

from colman_motion import follow_geometry

# Most tests use a flat chase (the camera level with the cube) to keep the numbers
# simple; the chase-camera tests at the end use the real elevation.
follow_twist = partial(
    follow_geometry.follow_twist,
    height=None,
    keep_level=True,
    min_radius=0.0,
    above=None,
)
goal_pose = partial(follow_geometry.goal_pose, height=None, min_radius=0.0, above=None)


def level_camera_looking_along(forward):
    """A camera looking along `forward` with the image level."""
    forward = np.asarray(forward, dtype=float)
    return level_rotation(forward / np.linalg.norm(forward))


def test_goal_pose_stands_off_on_the_line_of_sight():
    position, rotation = goal_pose(np.zeros(3), np.array([1.0, 0.0, 0.0]))
    assert np.allclose(position, [1.0 - STANDOFF_M, 0.0, 0.0])
    assert np.allclose(rotation[:, 2], [1.0, 0.0, 0.0])
    assert np.allclose(rotation[:, 1], [0.0, 0.0, -1.0])  # image y points down
    assert np.isclose(np.linalg.det(rotation), 1.0)


def test_goal_pose_has_a_level_image_when_aiming_up_and_down():
    for target in ([0.5, 0.3, 0.4], [0.5, -0.3, -0.4]):
        rotation = level_camera_looking_along(target)
        assert abs(rotation[2, 0]) < 1e-9  # image x axis horizontal


def test_level_is_undefined_when_looking_straight_up_or_down():
    assert level_rotation(np.array([0.0, 0.0, -1.0])) is None
    assert level_rotation(np.array([0.0, 0.0, 1.0])) is None


def test_no_motion_when_at_the_goal():
    cube = np.array([1.0, 0.0, 0.0])
    position, rotation = goal_pose(np.zeros(3), cube)
    linear, angular, distance_error, aim_error, roll_error = follow_twist(
        position, rotation, cube
    )
    assert np.allclose(linear, 0.0)
    assert np.allclose(angular, 0.0)
    assert abs(distance_error) < 1e-9 and aim_error < 1e-6 and abs(roll_error) < 1e-6


def test_too_far_moves_toward_the_cube_and_too_close_moves_away():
    rotation = level_camera_looking_along([1.0, 0.0, 0.0])
    far, *_ = follow_twist(np.zeros(3), rotation, np.array([0.5, 0.0, 0.0]))
    near, *_ = follow_twist(np.zeros(3), rotation, np.array([0.1, 0.0, 0.0]))
    assert far[0] > 0.0
    assert near[0] < 0.0


def test_cube_to_the_left_turns_the_camera_left():
    # Looking along +x with up +z, left is +y: a positive turn about +z.
    rotation = level_camera_looking_along([1.0, 0.0, 0.0])
    _, angular, _, aim_error, _ = follow_twist(
        np.zeros(3), rotation, np.array([0.2, 0.05, 0.0])
    )
    assert aim_error > 0.0
    assert angular[2] > 0.0
    assert abs(angular[0]) < 1e-9


def test_cube_above_tilts_the_camera_up():
    rotation = level_camera_looking_along([1.0, 0.0, 0.0])
    _, angular, *_ = follow_twist(np.zeros(3), rotation, np.array([0.2, 0.0, 0.05]))
    # Pitching a +x lens axis toward +z is a turn about -y.
    assert angular[1] < 0.0


def test_rolled_camera_rolls_back_without_moving():
    cube = np.array([0.2, 0.0, 0.0])
    rotation = level_camera_looking_along([1.0, 0.0, 0.0])
    angle = np.radians(20.0)
    c, s = np.cos(angle), np.sin(angle)
    roll_about_forward = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    rolled = roll_about_forward @ rotation
    linear, angular, _, aim_error, roll_error = follow_twist(np.zeros(3), rolled, cube)
    assert np.allclose(linear, 0.0)
    assert aim_error < 1e-6
    assert abs(abs(roll_error) - angle) < 1e-6
    # Turning about the line of sight, against the roll.
    assert np.dot(angular, [1.0, 0.0, 0.0]) * roll_error > 0.0


def test_speeds_are_capped():
    rotation = level_camera_looking_along([1.0, 0.0, 0.0])
    linear, angular, *_ = follow_twist(np.zeros(3), rotation, np.array([5.0, 3.0, 2.0]))
    assert np.linalg.norm(linear) <= MAX_LINEAR_MPS + 1e-9
    assert np.linalg.norm(angular) <= MAX_ANGULAR_RPS + 1e-9


def test_lowpass_and_clamp():
    assert np.allclose(lowpass(None, [1.0, 2.0, 3.0], 0.3), [1.0, 2.0, 3.0])
    assert np.allclose(
        lowpass([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.25), [0.25, 0.0, 0.0]
    )
    assert np.allclose(clamp_norm(np.array([3.0, 4.0, 0.0]), 1.0), [0.6, 0.8, 0.0])
    assert np.allclose(clamp_norm(np.array([0.3, 0.0, 0.0]), 1.0), [0.3, 0.0, 0.0])


def test_quaternion_round_trip():
    from colman_motion.follow_geometry import matrix_to_quat, quat_to_matrix

    for target in ([0.5, 0.3, 0.4], [-0.5, 0.3, -0.4], [0.0, -1.0, 0.2]):
        _, rotation = goal_pose(np.zeros(3), np.array(target))
        assert np.allclose(quat_to_matrix(*matrix_to_quat(rotation)), rotation)
    assert np.allclose(quat_to_matrix(0, 0, 0, 1), np.eye(3))


def test_twist_at_point_adds_the_turn_about_the_offset():
    from colman_motion.follow_geometry import twist_at_point

    # Turning about z at 1 rad/s moves a point 0.1 m along +x toward +y.
    moved = twist_at_point([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], np.zeros(3), [0.1, 0, 0])
    assert np.allclose(moved, [0.0, 0.1, 0.0])
    # A pure translation is the same everywhere.
    same = twist_at_point([0.1, 0.0, 0.0], np.zeros(3), np.zeros(3), [1.0, 2.0, 3.0])
    assert np.allclose(same, [0.1, 0.0, 0.0])
    # The conversion round-trips: camera -> tip -> camera.
    cam, tip = np.array([0.3, 0.1, 0.4]), np.array([0.3, 0.0, 0.3])
    lin, ang = np.array([0.05, -0.02, 0.01]), np.array([0.1, 0.2, -0.1])
    at_tip = twist_at_point(lin, ang, cam, tip)
    assert np.allclose(twist_at_point(at_tip, ang, tip, cam), lin)


def test_roll_takes_the_nearer_of_upright_and_upside_down():
    cube = np.array([0.2, 0.0, 0.0])
    upright = level_camera_looking_along([1.0, 0.0, 0.0])
    for degrees, expected in ((170.0, -10.0), (-170.0, 10.0), (20.0, 20.0)):
        angle = np.radians(degrees)
        c, s = np.cos(angle), np.sin(angle)
        rolled = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ upright
        *_, roll_error = follow_twist(np.zeros(3), rolled, cube)
        assert abs(abs(roll_error) - abs(np.radians(expected))) < 1e-6, degrees


def test_goal_pose_keeps_an_upside_down_camera_upside_down():
    upright = level_camera_looking_along([1.0, 0.0, 0.0])
    flipped_x = -upright[:, 0]
    _, rotation = goal_pose(np.zeros(3), np.array([1.0, 0.0, 0.0]), camera_x=flipped_x)
    assert np.dot(rotation[:, 0], flipped_x) > 0.99
    assert np.isclose(np.linalg.det(rotation), 1.0)


def test_roll_correction_fades_out_near_vertical():
    # Camera looking almost straight down, rolled 30 degrees from level.
    def roll_rate(tilt_sin):
        forward = np.array([tilt_sin, 0.0, -np.sqrt(1.0 - tilt_sin**2)])
        right = np.cross(forward, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        level = np.column_stack([right, np.cross(forward, right), forward])
        angle = np.radians(30.0)
        c, s = np.cos(angle), np.sin(angle)
        rolled = level @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        _, angular, *_ = follow_twist(np.zeros(3), rolled, forward)
        return np.linalg.norm(angular)

    assert roll_rate(0.05) == 0.0
    assert 0.0 < roll_rate(0.2) < roll_rate(0.5)


def test_chase_position_is_straight_above_the_cube():
    cube = np.array([0.40, 0.10, 0.03])
    assert np.allclose(chase_position(cube), [0.40, 0.10, 0.33])


def test_chase_height_follows_the_cube_not_the_table():
    for z in (0.0, 0.03, 0.08):
        assert np.isclose(chase_position(np.array([0.35, 0.1, z]))[2], z + ABOVE_CUBE_M)


def test_cube_moving_sideways_moves_the_camera_not_just_turns_it():
    cube = np.array([0.4, 0.0, 0.03])
    position, rotation = follow_geometry.goal_pose(
        np.zeros(3), cube, camera_x=np.array([1.0, 0.0, 0.0]), cube_axis=cube_edge(0.0)
    )
    assert np.allclose(follow_geometry.follow_twist(position, rotation, cube)[0], 0.0)
    moved = cube + np.array([0.0, 0.10, 0.0])
    linear, *_ = follow_geometry.follow_twist(position, rotation, moved)
    assert linear[1] > 0.0  # the camera translates toward the cube's new side
    assert abs(linear[2]) < 1e-9  # and stays at its height above the cube


def test_camera_rises_when_the_cube_is_lifted():
    cube = np.array([0.4, 0.0, 0.03])
    position, rotation = follow_geometry.goal_pose(
        np.zeros(3), cube, camera_x=np.array([1.0, 0.0, 0.0]), cube_axis=cube_edge(0.0)
    )
    linear, *_ = follow_geometry.follow_twist(
        position, rotation, cube + np.array([0.0, 0.0, 0.10])
    )
    assert linear[2] > 0.0


def looking_down():
    """A camera looking straight down with its image x axis along +x."""
    return np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def cube_edge(degrees):
    angle = np.radians(degrees)
    return np.array([np.cos(angle), np.sin(angle), 0.0])


def roll_twist(cube_degrees, wrist_roll=None):
    cube = np.array([0.4, 0.0, 0.03])
    camera = chase_position(cube)
    return follow_geometry.follow_twist(
        camera,
        looking_down(),
        cube,
        cube_axis=cube_edge(cube_degrees),
        wrist_roll=wrist_roll,
    )


def test_camera_turns_with_the_cube():
    _, aligned, *_ = roll_twist(0.0)
    assert np.allclose(aligned, 0.0)
    _, counter_clockwise, _, _, roll = roll_twist(20.0)
    # The cube turned 20 degrees counter-clockwise seen from above: so does the camera.
    assert counter_clockwise[2] > 0.0
    assert np.isclose(roll, np.radians(-20.0))  # about the downward lens axis
    _, clockwise, *_ = roll_twist(-20.0)
    assert clockwise[2] < 0.0


def test_a_cube_looks_the_same_every_90_degrees():
    for degrees in (90.0, 180.0, -90.0):
        _, angular, *_ = roll_twist(degrees)
        assert np.allclose(angular, 0.0), degrees
    # 70 degrees is the same as -20 degrees.
    assert np.allclose(roll_twist(70.0)[1], roll_twist(-20.0)[1])


def test_the_camera_takes_the_edge_that_spares_the_wrist():
    # 30 degrees one way or 60 the other: with the wrist already turned +1.2 rad, take
    # the turn that brings it back.
    _, plain, _, _, plain_roll = roll_twist(-30.0)
    _, spared, _, _, spared_roll = roll_twist(-30.0, wrist_roll=-1.2)
    assert np.isclose(plain_roll, np.radians(30.0))
    assert np.isclose(spared_roll, np.radians(30.0))
    _, spared, _, _, spared_roll = roll_twist(-30.0, wrist_roll=1.2)
    assert np.isclose(spared_roll, np.radians(-60.0))


def test_no_roll_command_when_not_looking_down():
    cube = np.array([0.4, 0.0, 0.03])
    camera = np.array([0.1, 0.0, 0.03])  # level with the cube, looking along +x
    rotation = level_camera_looking_along([1.0, 0.0, 0.0])
    *_, roll_error = follow_geometry.follow_twist(
        camera, rotation, cube, cube_axis=cube_edge(20.0)
    )
    assert roll_error == 0.0


def test_goal_pose_lines_the_image_up_with_the_cube():
    cube = np.array([0.4, 0.0, 0.03])
    _, rotation = follow_geometry.goal_pose(
        np.zeros(3), cube, camera_x=np.array([1.0, 0.0, 0.0]), cube_axis=cube_edge(20.0)
    )
    assert np.allclose(rotation[:, 0], cube_edge(20.0), atol=1e-9)
    assert np.allclose(rotation[:, 2], [0.0, 0.0, -1.0])
    assert np.isclose(np.linalg.det(rotation), 1.0)


def test_horizontal_axis_from_a_tag_on_any_face():
    def yaw(degrees):
        c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    # A tag on the top face: its x axis is horizontal.
    assert np.allclose(horizontal_axis(yaw(30.0)), cube_edge(30.0))
    # A tag on a side face, turned so its x axis points up: its y axis is horizontal.
    side = yaw(30.0) @ np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
    assert np.isclose(side[2, 0], 1.0)  # x axis up
    axis = horizontal_axis(side)
    assert np.isclose(abs(np.dot(axis, cube_edge(30.0))), 1.0) or np.isclose(
        abs(np.dot(axis, cube_edge(120.0))), 1.0
    )


def test_fourfold_smoothing_survives_the_90_degree_wrap():
    mean = axis_from_fourfold(
        0.5 * (fourfold(cube_edge(44.0)) + fourfold(cube_edge(-44.0)))
    )
    # 44 and -44 degrees are 2 degrees apart on a cube (88 -> -92 ... 46 vs 44 mod 90).
    angle = np.degrees(np.arctan2(mean[1], mean[0]))
    assert abs(((angle + 45.0) % 90.0) - 45.0) > 40.0  # near +-45, not near 0
    assert np.allclose(axis_from_fourfold(fourfold(cube_edge(10.0))), cube_edge(10.0))


def test_roll_is_left_alone_by_default():
    cube = np.array([0.2, 0.0, 0.0])
    upright = level_camera_looking_along([1.0, 0.0, 0.0])
    angle = np.radians(40.0)
    c, s = np.cos(angle), np.sin(angle)
    rolled = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ upright
    camera = np.zeros(3)
    _, angular, _, _, roll_error = follow_geometry.follow_twist(
        camera, rolled, cube, height=None
    )
    assert np.allclose(angular, 0.0)  # looking right at the cube: no turn at all
    assert roll_error == 0.0


def test_aiming_does_not_spin_the_camera_about_its_lens_axis():
    rotation = level_camera_looking_along([1.0, 0.0, 0.0])
    cube = np.array([0.2, 0.05, 0.03])
    _, angular, *_ = follow_geometry.follow_twist(
        np.zeros(3), rotation, cube, height=None
    )
    lens_axis = rotation[:, 2]
    assert abs(np.dot(angular, lens_axis)) < 1e-9
    assert np.linalg.norm(angular) > 0.0


def test_limit_avoidance_turns_a_wrist_back_toward_zero():
    from colman_motion.follow_geometry import limit_avoidance

    axis = np.array([0.0, 0.0, -1.0])
    assert np.allclose(limit_avoidance(0.5, axis, 0.7), 0.0)  # inside the soft zone
    assert np.allclose(limit_avoidance(-0.7, axis, 0.7), 0.0)
    # Positive angle: turn negatively about the axis (and the reverse), and harder
    # the further out it is.
    near = limit_avoidance(0.8, axis, 0.7)
    far = limit_avoidance(0.95, axis, 0.7)
    assert np.dot(near, axis) < 0.0 < np.dot(near, -axis)
    assert np.linalg.norm(far) > np.linalg.norm(near)
    assert np.allclose(limit_avoidance(-0.8, axis, 0.7), -near)
    assert np.linalg.norm(limit_avoidance(1.57, axis, 0.7)) <= 0.5 + 1e-9


def test_camera_never_goes_closer_than_30_cm_to_the_base_axis():
    for angle in (0.0, 0.8, 2.0, -2.5):
        for radius in (0.0, 0.1, 0.2, 0.3, 0.4):
            cube = np.array([radius * np.cos(angle), radius * np.sin(angle), 0.03])
            position = chase_position(cube)
            assert np.linalg.norm(position[:2]) >= 0.30 - 1e-9
            assert np.isclose(position[2], 0.03 + ABOVE_CUBE_M)


def test_a_cube_near_the_base_is_viewed_from_beyond_it():
    cube = np.array([0.0, 0.15, 0.03])
    position = chase_position(cube)
    assert np.allclose(position[:2], [0.0, 0.30])  # same direction, 30 cm out
    goal = follow_geometry.goal_pose(np.zeros(3), cube)
    assert goal is not None
    assert goal[1][1, 2] < 0.0  # still looking toward the cube, along -y


def test_goal_is_never_beyond_the_arms_reach():
    for cube in (
        [1.0, 0.0, 0.03],
        [0.6, -0.5, 0.03],
        [0.0, -0.9, 0.10],
        [0.4, 0.0, 0.25],
    ):
        goal = chase_position(np.array(cube))
        assert np.linalg.norm(goal - SHOULDER_POSITION) <= MAX_REACH_M + 1e-9


def test_a_far_cube_is_still_chased_in_its_direction_and_aimed_at():
    cube = np.array([1.0, 0.0, 0.03])
    goal = chase_position(cube)
    assert abs(goal[1]) < 1e-9 and goal[0] > 0.30  # same bearing, as far as it can go
    # The camera at that goal still turns to look at the cube beyond it.
    rotation = looking_down()
    _, angular, _, aim_error, _ = follow_geometry.follow_twist(goal, rotation, cube)
    assert aim_error > np.radians(30.0)
    assert np.linalg.norm(angular) > 0.0


def test_a_cube_in_reach_is_not_moved_by_the_reach_limit():
    cube = np.array([0.4, 0.1, 0.03])
    assert np.allclose(chase_position(cube), chase_position(cube, max_reach=None))


def test_elbow_avoidance_pulls_the_camera_in_when_the_arm_is_nearly_straight():
    from colman_motion.follow_geometry import elbow_avoidance

    camera = np.array([0.38, 0.0, 0.515])
    assert np.allclose(elbow_avoidance(-1.2, camera), 0.0)  # well bent: nothing
    assert np.allclose(elbow_avoidance(0.7, camera), 0.0)  # at the edge of the zone
    inward = SHOULDER_POSITION - camera
    inward /= np.linalg.norm(inward)
    slight = elbow_avoidance(-0.5, camera)
    straight = elbow_avoidance(-0.1, camera)
    assert np.allclose(slight / np.linalg.norm(slight), inward)  # toward the shoulder
    assert np.linalg.norm(straight) > np.linalg.norm(slight)
    assert np.linalg.norm(elbow_avoidance(0.0, camera)) <= 0.08 + 1e-9
    assert np.allclose(
        elbow_avoidance(0.2, camera) / 1.0, elbow_avoidance(-0.2, camera)
    )
