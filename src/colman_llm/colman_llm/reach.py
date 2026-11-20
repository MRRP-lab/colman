"""Can the arm reach a cube? A kinematic check the node makes before it moves anything.

Stacking can ask for a cube the arm cannot reach, or a spot on a stack that is too far
or too high. Without a check the arm picks the cube up and only then finds out that it
cannot place it, and is left holding it. This module answers the question first.

It is a model of the UR3e with the vacuum tool, not the motion server's solver:
    * the same joint chain, joint limits and tool tip as the robot description,
    * the tool pointing straight down, tried at the same quarter turns the motion
      server tries (``TOOL_YAWS_DEG``),
    * a pick or place needs both its low pose and the approach pose above it, on one
      IK branch, as ``vertical_pair`` does.
It does not know about collisions, so it only says "the arm cannot get there", never
"the way is clear". Its edge is good to about 1 to 2 cm. A pick is allowed if any point
within ``PICK_SLACK_M`` is reachable, because a pick the arm cannot do is refused by
the motion server before anything moves. A place has to be reachable with
``PLACE_MARGIN_M`` to spare, because a failed place leaves the cube in the gripper.

The numbers below are copies of the robot description, and ``test_reach.py`` fails if
they drift from ``ur_config.yaml``, ``joint_limits.yaml``, the xacro or the motion
server.
"""

import math
from functools import lru_cache

import numpy as np
from colman_description.workcell import CUBE_SIZE_M, cube_top_z

# Offsets the motion server uses (motion_server.py). The pick goes EE_OFFSET_M below the
# tag and approaches APPROACH_OFFSET_M above it. A place drops DROP_OFFSET_M above the
# tag of the cube it goes on and approaches PLACE_APPROACH_OFFSET_M above that.
EE_OFFSET_M = 0.01
APPROACH_OFFSET_M = 0.1
DROP_OFFSET_M = CUBE_SIZE_M
PLACE_APPROACH_OFFSET_M = 0.05
TOOL_YAWS_DEG = (0, 180, 90, -90)  # arm_control.py

PICK_SLACK_M = 0.015
PLACE_MARGIN_M = 0.01
# The arm folds against itself and its base close in. The kinematic model is too
# optimistic there and does not model the collision, so nothing within this distance of
# the base counts. 0.16 m is where docs/arm_reach, which was checked more carefully,
# finds the nearest reachable cube along the forward axis.
MIN_RADIUS_M = 0.16

# The factory-measured link offsets of the arm, from ur_config.yaml, as
# (x, y, z, roll, pitch, yaw) for shoulder, upper arm, forearm and wrist 1 to 3.
LINKS = (
    (0, 0, 0.15192283047976676, 0, 0, -5.7268099108373494e-08),
    (-0.00019515468864545848, 0, 0, 1.5704735255000193, 0, -1.8483950676717602e-06),
    (
        -0.24394101790978492,
        0,
        0,
        3.140178102122266,
        3.1400932105845056,
        3.141582941473686,
    ),
    (
        -0.21325457546021262,
        -0.0005229410967979574,
        0.13132703065685788,
        0.003981955049675383,
        0.000999715507256646,
        4.149797461644216e-06,
    ),
    (
        3.809893732655968e-05,
        -0.08524087311177547,
        5.511576546356224e-05,
        1.5701497384180927,
        0,
        -1.1522983195307317e-06,
    ),
    (
        -8.127062371569708e-05,
        0.0922699310800553,
        -0.0002040456792284709,
        1.5685849309021394,
        3.141592653589793,
        -3.1415925482156024,
    ),
)
# Fixed transforms before the shoulder (base_link to base_link_inertia) and after wrist
# 3 (wrist_3-flange, flange-tool0), as (x, y, z, roll, pitch, yaw).
BASE = (0, 0, 0, 0, 0, math.pi)
FLANGE = (0, 0, 0, 0, -math.pi / 2, -math.pi / 2)
TOOL0 = (0, 0, 0, math.pi / 2, 0, math.pi / 2)
# From tool0 to the vacuum tip: the camera adapter and mount (0.02 m up) and the nozzle
# tip's offset in colman.urdf.xacro.
MOUNT = (0, 0, 0.02, 0, 0, 0)
TIP = (0, 0.04552, 0.068, 0, 0, 0)
# Joint limits (rad) from colman_moveit_config/config/joint_limits.yaml.
LIMITS = (
    (-2 * math.pi, 2 * math.pi),  # shoulder pan
    (-math.pi, 0.0),  # shoulder lift
    (-math.pi, math.pi),  # elbow
    (-4.71238898038469, 1.5707963267948966),  # wrist 1
    (-2.9146998508305302, 2.9146998508305302),  # wrist 2
    (-2.2689280275926285, 2.443460952792061),  # wrist 3
)
HOME = (0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0)  # poses.py HOME_POSE

SEEDS = 16
ITERATIONS = 100
POSITION_TOL_M = 1e-3
ORIENTATION_TOL = 0.01


def _rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _transform(values):
    x, y, z, roll, pitch, yaw = values
    matrix = np.eye(4)
    matrix[:3, :3] = _rotation(roll, pitch, yaw)
    matrix[:3, 3] = (x, y, z)
    return matrix


def _compose(*transforms):
    matrix = np.eye(4)
    for each in transforms:
        matrix = matrix @ each
    return matrix


_BEFORE = _transform(BASE)
_ORIGINS = [_transform(link) for link in LINKS]
_AFTER = _compose(
    _transform(FLANGE), _transform(TOOL0), _transform(MOUNT), _transform(TIP)
)
_LOW = np.array([low for low, _ in LIMITS])
_HIGH = np.array([high for _, high in LIMITS])
# Tool pointing straight down is a half turn about x: (qx, qy, qz, qw) = (1, 0, 0, 0).
_DOWN = np.diag([1.0, -1.0, -1.0])


def forward(q):
    """Tip positions, rotations, joint axes and joint origins for joints q (B, 6)."""
    batch = q.shape[0]
    frame = np.broadcast_to(_BEFORE, (batch, 4, 4)).copy()
    axes, origins = [], []
    for i in range(6):
        frame = frame @ _ORIGINS[i]
        axes.append(frame[:, :3, 2].copy())
        origins.append(frame[:, :3, 3].copy())
        c, s = np.cos(q[:, i]), np.sin(q[:, i])
        turn = np.zeros((batch, 4, 4))
        turn[:, 0, 0], turn[:, 0, 1], turn[:, 1, 0], turn[:, 1, 1] = c, -s, s, c
        turn[:, 2, 2] = turn[:, 3, 3] = 1.0
        frame = frame @ turn
    frame = frame @ _AFTER
    return frame[:, :3, 3], frame[:, :3, :3], np.stack(axes, 1), np.stack(origins, 1)


def solve(targets, orientations, seeds):
    """Damped least squares IK for B poses at once. Returns (joints, solved)."""
    q = np.clip(seeds.copy(), _LOW, _HIGH)
    for _ in range(ITERATIONS):
        position, rotation, axes, origins = forward(q)
        error = np.concatenate(
            [targets - position, _batch_orientation_error(orientations, rotation)], 1
        )
        jac = np.concatenate(
            [np.cross(axes, position[:, None, :] - origins), axes], 2
        ).transpose(0, 2, 1)
        damped = jac @ jac.transpose(0, 2, 1) + 0.02**2 * np.eye(6)
        step = (jac.transpose(0, 2, 1) @ np.linalg.solve(damped, error[:, :, None]))[
            :, :, 0
        ]
        q = np.clip(q + np.clip(step, -0.3, 0.3), _LOW, _HIGH)
    position, rotation, _, _ = forward(q)
    pos_err = np.linalg.norm(targets - position, axis=1)
    ori_err = np.linalg.norm(_batch_orientation_error(orientations, rotation), axis=1)
    return q, (pos_err < POSITION_TOL_M) & (ori_err < ORIENTATION_TOL)


def _batch_orientation_error(orientations, rotation):
    err = orientations @ rotation.transpose(0, 2, 1)
    return 0.5 * np.stack(
        [
            err[:, 2, 1] - err[:, 1, 2],
            err[:, 0, 2] - err[:, 2, 0],
            err[:, 1, 0] - err[:, 0, 1],
        ],
        axis=1,
    )


def _yaw_matrix(degrees):
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


_YAWS = np.stack([_yaw_matrix(d) @ _DOWN for d in TOOL_YAWS_DEG])
_SEEDS = np.concatenate(
    [
        np.array([HOME]),
        _LOW + (_HIGH - _LOW) * np.random.default_rng(0).random((SEEDS, 6)),
    ]
)


def pair_reachable(x, y, z_low, z_high):
    """True if the tip can be at (x, y, z_low) and (x, y, z_high), tool straight down.

    The low pose is solved from several starting postures and the high pose from each
    solution, so both are on one IK branch. Any tool quarter turn may do it.
    """
    seeds = np.tile(_SEEDS, (len(_YAWS), 1))
    yaws = np.repeat(_YAWS, len(_SEEDS), axis=0)
    low = np.tile([x, y, z_low], (len(seeds), 1))
    high = np.tile([x, y, z_high], (len(seeds), 1))
    q_low, ok_low = solve(low, yaws, seeds)
    if not ok_low.any():
        return False
    _, ok_high = solve(high[ok_low], yaws[ok_low], q_low[ok_low])
    return bool(ok_high.any())


@lru_cache(maxsize=4096)
def _reachable_mm(x_mm, y_mm, low_mm, high_mm):
    return pair_reachable(x_mm / 1000, y_mm / 1000, low_mm / 1000, high_mm / 1000)


def _reachable(x, y, z_low, z_high):
    return _reachable_mm(*(round(v * 1000) for v in (x, y, z_low, z_high)))


def _around(x, y, distance):
    return [
        (x + dx, y + dy)
        for dx, dy in (
            (0, 0),
            (distance, 0),
            (-distance, 0),
            (0, distance),
            (0, -distance),
        )
    ]


def level_tag_z(level):
    """z of the tag on top of the cube at this level (1 is on the table)."""
    return cube_top_z(level)


def _too_close(x, y):
    return math.hypot(x, y) < MIN_RADIUS_M


def can_pick(x, y, tag_z):
    """Can the arm pick the cube whose tag is at (x, y, tag_z)? Lenient at the edge."""
    if _too_close(x, y):
        return False
    low, high = tag_z - EE_OFFSET_M, tag_z + APPROACH_OFFSET_M
    return any(_reachable(px, py, low, high) for px, py in _around(x, y, PICK_SLACK_M))


def can_place_on(x, y, tag_z):
    """Can the arm place a cube on the cube whose tag is at (x, y, tag_z)? Strict."""
    if _too_close(x, y):
        return False
    low = tag_z + DROP_OFFSET_M
    high = low + PLACE_APPROACH_OFFSET_M
    return all(
        _reachable(px, py, low, high) for px, py in _around(x, y, PLACE_MARGIN_M)
    )
