"""Singularity-robust differential IK for the UR3e (numpy only).

Forward kinematics follows the ur_description URDF chain, so poses are in the
ROS ``base_link`` frame (the UR base rotated by pi about z) and match TF.
"""

import numpy as np

_HALF_PI = np.pi / 2.0

# (xyz, rpy) of each joint origin in the URDF, after base_link -> base_link_inertia.
_JOINT_ORIGINS = (
    ((0.0, 0.0, 0.15185), (0.0, 0.0, 0.0)),  # shoulder_pan
    ((0.0, 0.0, 0.0), (_HALF_PI, 0.0, 0.0)),  # shoulder_lift
    ((-0.24355, 0.0, 0.0), (0.0, 0.0, 0.0)),  # elbow
    ((-0.2132, 0.0, 0.13105), (0.0, 0.0, 0.0)),  # wrist_1
    ((0.0, -0.08535, 0.0), (_HALF_PI, 0.0, 0.0)),  # wrist_2
    ((0.0, 0.0921, 0.0), (_HALF_PI, np.pi, np.pi)),  # wrist_3
)
# wrist_3 -> flange -> tool0, and base_link -> base_link_inertia.
_FLANGE_RPY = (0.0, -_HALF_PI, -_HALF_PI)
_TOOL0_RPY = (_HALF_PI, 0.0, _HALF_PI)
_BASE_RPY = (0.0, 0.0, np.pi)

DEFAULT_LOWER = np.array([-2 * np.pi, -np.pi, -np.pi, -1.5 * np.pi, -2.618, -np.pi / 2])
DEFAULT_UPPER = np.array([2 * np.pi, 0.0, np.pi, np.pi / 2, 2.618, np.pi / 2])

# Damping tuned on the UR3e (see the tests): ordinary poses are undamped, straight
# elbow / wrist_2 = 0 poses are damped.
LAMBDA_MAX = 0.1
W0 = 0.005


def _rpy(r, p, y):
    """Rotation matrix for URDF fixed-axis roll, pitch, yaw."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _chain(q):
    """Joint axes and origins in base_link, plus the tool0 position/rotation."""
    q = np.asarray(q, dtype=float)
    rot = _rpy(*_BASE_RPY)
    pos = np.zeros(3)
    axes, origins = [], []
    for angle, (xyz, rpy) in zip(q, _JOINT_ORIGINS):
        pos = pos + rot @ np.array(xyz)
        rot = rot @ _rpy(*rpy)
        axes.append(rot[:, 2].copy())
        origins.append(pos.copy())
        rot = rot @ _rot_z(angle)
    rot = rot @ _rpy(*_FLANGE_RPY) @ _rpy(*_TOOL0_RPY)
    return axes, origins, pos, rot


def tool_pose(q):
    """Return (position (3,), rotation (3, 3)) of tool0 in base_link."""
    _, _, pos, rot = _chain(q)
    return pos, rot


def jacobian(q, point_in_tool0=(0.0, 0.0, 0.0)):
    """6x6 geometric Jacobian (linear rows, then angular) in base_link.

    The linear part is for a point fixed to tool0 at `point_in_tool0` (tool0 frame).
    """
    axes, origins, pos, rot = _chain(q)
    point = pos + rot @ np.asarray(point_in_tool0, dtype=float)
    jac = np.zeros((6, 6))
    for i, (axis, origin) in enumerate(zip(axes, origins)):
        jac[:3, i] = np.cross(axis, point - origin)
        jac[3:, i] = axis
    return jac


def manipulability(jac):
    """Yoshikawa manipulability sqrt(det(J J^T))."""
    return float(np.sqrt(max(np.linalg.det(jac @ jac.T), 0.0)))


def damping_squared(w, w0=W0, lambda_max=LAMBDA_MAX):
    """Nakamura-Hanafusa adaptive damping term lambda^2."""
    if w >= w0:
        return 0.0
    return lambda_max**2 * (1.0 - (w / w0) ** 2)


def limit_joint_velocity(q_dot, q, lower, upper, margin=0.15):
    """Scale components heading into a joint limit to 0 across the soft margin.

    Components moving away from a limit are untouched; a joint already past a limit
    can only move back.
    """
    q_dot = np.array(q_dot, dtype=float)
    q = np.asarray(q, dtype=float)
    to_upper = (np.asarray(upper, dtype=float) - q) / margin
    to_lower = (q - np.asarray(lower, dtype=float)) / margin
    up = (q_dot > 0.0) & (to_upper < 1.0)
    q_dot[up] *= np.clip(to_upper[up], 0.0, 1.0)
    down = (q_dot < 0.0) & (to_lower < 1.0)
    q_dot[down] *= np.clip(to_lower[down], 0.0, 1.0)
    return q_dot


def dls_joint_velocity(
    q,
    twist,
    point_in_tool0=(0.0, 0.0, 0.0),
    max_joint_velocity=None,
    lower=DEFAULT_LOWER,
    upper=DEFAULT_UPPER,
    margin=0.15,
    w0=W0,
    lambda_max=LAMBDA_MAX,
):
    """Joint velocities (rad/s) giving `twist` (m/s, rad/s, base_link) of the point.

    Uses damped least squares with adaptive damping near singularities.
    `max_joint_velocity` (scalar or per joint) rescales the whole vector. Pass
    lower/upper = None to skip joint limit handling. Always returns finite values;
    bad input gives zeros.
    """
    zeros = np.zeros(6)
    try:
        q = np.asarray(q, dtype=float).reshape(6)
        twist = np.asarray(twist, dtype=float).reshape(6)
        point = np.asarray(point_in_tool0, dtype=float).reshape(3)
    except (TypeError, ValueError):
        return zeros
    if not (np.isfinite(q).all() and np.isfinite(twist).all()):
        return zeros
    if not np.isfinite(point).all():
        return zeros

    jac = jacobian(q, point)
    jjt = jac @ jac.T
    lam2 = damping_squared(manipulability(jac), w0, lambda_max)
    try:
        q_dot = jac.T @ np.linalg.solve(jjt + lam2 * np.eye(6), twist)
    except np.linalg.LinAlgError:
        return zeros
    if not np.isfinite(q_dot).all():
        return zeros

    if max_joint_velocity is not None:
        ratio = np.max(np.abs(q_dot) / np.asarray(max_joint_velocity, dtype=float))
        if ratio > 1.0:
            q_dot = q_dot / ratio
    if lower is not None and upper is not None:
        q_dot = limit_joint_velocity(q_dot, q, lower, upper, margin)
    return np.nan_to_num(q_dot, nan=0.0, posinf=0.0, neginf=0.0)
