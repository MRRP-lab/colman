"""Rotation helpers for orienting the hand."""

import math

import numpy as np


def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def yawed(orientation, degrees):
    """The orientation (x, y, z, w) turned by these degrees about the vertical axis.

    The suction cup is round, so a tool pointing straight down can be turned about the
    vertical axis without changing the grasp. It moves the arm's wrist, which sticks out
    to one side of the tool, and that is how a pick next to a tall stack finds a free
    way in.
    """
    half = math.radians(degrees) / 2.0
    ax, ay, az, aw = 0.0, 0.0, math.sin(half), math.cos(half)
    bx, by, bz, bw = orientation
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def level_camera_rotation(direction):
    """
    Camera rotation (columns x right, y down, z forward) looking along direction
    with the image's up side toward world up. Looking straight down, up is +x.
    """
    down = np.array([0.0, 0.0, -1.0])
    if abs(np.dot(direction, down)) > 0.99:
        down = np.array([-1.0, 0.0, 0.0])
    y = down - np.dot(down, direction) * direction
    y /= np.linalg.norm(y)
    x = np.cross(y, direction)
    return np.column_stack([x, y, direction])


def rotation_to_quaternion(x, y, z):
    m00, m10, m20 = x
    m01, m11, m21 = y
    m02, m12, m22 = z
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


def yaw_of(q):
    """Degrees the x axis of orientation q (x, y, z, w) points around the vertical."""
    m = quaternion_to_matrix(q)
    return math.degrees(math.atan2(m[1, 0], m[0, 0]))


def quarter_turn(degrees):
    """degrees folded into [-45, 45): a cube looks the same after a quarter turn."""
    return (degrees + 45.0) % 90.0 - 45.0


def cube_yaw_in_tool(tag_q, tool_q):
    """How far a held cube is turned from the tool, as a fraction of a quarter turn."""
    return quarter_turn(yaw_of(tag_q) - yaw_of(tool_q))


def tool_yaw_to_square(dest_q, cube_in_tool):
    """Tool turn that puts a cube held at cube_in_tool square with the tag dest_q."""
    return quarter_turn(yaw_of(dest_q) - cube_in_tool)
