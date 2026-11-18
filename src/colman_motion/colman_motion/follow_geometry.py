"""Pure geometry for the cube follower: where to aim and how fast to turn.

Everything is in one fixed frame (base_link). The camera frame is the optical one:
+z forward out of the lens, +x right in the image, +y down in the image.
"""

import numpy as np

UP = np.array([0.0, 0.0, 1.0])

STANDOFF_M = 0.20
# The camera chases the cube like a third-person game camera: it stays at a fixed
# height above the table (the z = 0 plane of base_link) and STANDOFF_M from the cube,
# on the robot's side of it.
CAMERA_HEIGHT_M = 0.30
# The camera hangs this far directly above the cube, looking down at it, and moves
# with it (this replaces the fixed-height chase above when `above` is given).
ABOVE_CUBE_M = 0.30
# The least the camera stays back from the cube horizontally, if the cube is so high
# that the standoff leaves no room.
MIN_HORIZONTAL_OFFSET_M = 0.05
# Where the UR3e's shoulder is, and how far the camera can be from it before the
# arm is stretched straight. Servo stops for the elbow singularity when the elbow is
# nearly straight (seen at 0.64 m and again at 0.53 m from the shoulder), so the goal
# never goes further out than this.
SHOULDER_POSITION = np.array([0.0, 0.0, 0.152])
MAX_REACH_M = 0.48
# Closer to a straight elbow than this, the camera is pulled back toward the shoulder.
ELBOW_SOFT_RAD = 0.7
# The camera never goes closer than this to the base's vertical axis.
MIN_CAMERA_RADIUS_M = 0.30
# No motion while the distance and the aim are this close to the goal, so detection
# noise doesn't make the arm jitter.
DISTANCE_DEADBAND_M = 0.015
AIM_DEADBAND_RAD = np.radians(1.5)
ROLL_DEADBAND_RAD = np.radians(2.0)
LINEAR_GAIN = 1.5  # 1/s: speed per metre of distance error
ANGULAR_GAIN = 1.5  # 1/s: rate per radian of aim error
MAX_LINEAR_MPS = 0.10
MAX_ANGULAR_RPS = 0.4
# Below this the camera looks almost straight up or down and "level" has no meaning.
MIN_HORIZONTAL_SIN = 0.1
FULL_ROLL_SIN = 0.4


def lowpass(previous, new, alpha):
    """Exponential smoothing; `previous` of None starts from `new`."""
    if previous is None:
        return np.asarray(new, dtype=float)
    return (1.0 - alpha) * np.asarray(previous) + alpha * np.asarray(new)


def twist_at_point(linear, angular, from_position, to_position):
    """
    The velocity of the point `to_position` on a rigid body that moves with `linear`
    at `from_position` and turns at `angular`. Servo moves the arm's tip link and
    turns it about that link, while the follower works out the camera's motion.
    """
    offset = np.asarray(to_position) - np.asarray(from_position)
    return np.asarray(linear) + np.cross(angular, offset)


def limit_avoidance(angle, axis, soft_rad, gain=ANGULAR_GAIN, max_rate=0.5):
    """
    Angular velocity that turns a rotary joint back toward zero once it is further
    than `soft_rad` from it, about its rotation `axis` in the base frame; zero inside
    the soft zone. The camera's roll about its lens axis is free, so spinning the last
    wrist joint there does not disturb the aim.
    """
    excess = abs(angle) - soft_rad
    if excess <= 0.0:
        return np.zeros(3)
    rate = min(max_rate, gain * excess)
    return -np.sign(angle) * rate * np.asarray(axis, dtype=float)


def elbow_avoidance(
    elbow_angle,
    camera_position,
    soft_rad=ELBOW_SOFT_RAD,
    gain=0.15,
    max_rate=0.08,
):
    """
    Velocity that pulls the camera in toward the shoulder when the elbow is closer to
    straight (angle 0) than `soft_rad`: the harder, the straighter. Shortening the
    reach is how the arm leaves the stretched-out singularity, so this keeps it from
    getting there, and gets it away if it has. Zero when the elbow is bent enough.
    """
    excess = soft_rad - abs(elbow_angle)
    if excess <= 0.0:
        return np.zeros(3)
    inward = SHOULDER_POSITION - np.asarray(camera_position, dtype=float)
    length = np.linalg.norm(inward)
    if length < 1e-6:
        return np.zeros(3)
    return min(max_rate, gain * excess) * inward / length


def clamp_norm(vector, limit):
    """Scale `vector` down to length `limit` if it is longer, keeping its direction."""
    norm = np.linalg.norm(vector)
    if norm <= limit or norm == 0.0:
        return vector
    return vector * (limit / norm)


def chase_position(
    cube_position,
    standoff=STANDOFF_M,
    height=CAMERA_HEIGHT_M,
    min_radius=MIN_CAMERA_RADIUS_M,
    above=ABOVE_CUBE_M,
    max_reach=MAX_REACH_M,
):
    """
    Where the camera should be: `above` the cube, straight over it, so it follows the
    cube around. With above None it is instead at `height` above the table, `standoff`
    from the cube, on the base's side of it (level with the cube if height is None).
    Either way it stays at least `min_radius` from the base axis, and with `above`
    no further than `max_reach` from the shoulder (None for no limit).
    """
    cube = np.asarray(cube_position, dtype=float)
    if above is not None:
        xy = cube[:2]
        if np.linalg.norm(xy) < min_radius:
            outward = xy / np.linalg.norm(xy) if np.linalg.norm(xy) > 1e-6 else None
            xy = min_radius * (outward if outward is not None else np.array([1.0, 0.0]))
        goal = np.array([xy[0], xy[1], cube[2] + above])
        if max_reach is not None:
            offset = goal - SHOULDER_POSITION
            distance = np.linalg.norm(offset)
            if distance > max_reach:
                goal = SHOULDER_POSITION + offset * (max_reach / distance)
        return goal
    toward_base = -np.array([cube[0], cube[1], 0.0])
    length = np.linalg.norm(toward_base)
    if length < 1e-6:
        toward_base = np.array([-1.0, 0.0, 0.0])
    else:
        toward_base /= length
    if height is None:
        height = cube[2]
    rise = height - cube[2]
    back = np.sqrt(max(standoff**2 - rise**2, MIN_HORIZONTAL_OFFSET_M**2))
    xy = cube[:2] + back * toward_base[:2]
    # Close to the base the arm has to fold up tightly, which is a singular pose that
    # Servo refuses to move in. Keep the camera at least this far from the base axis,
    # on the cube's side: beyond the cube if it is that close to the base.
    if np.linalg.norm(xy) < min_radius:
        if np.linalg.norm(cube[:2]) > 1e-6:
            outward = cube[:2] / np.linalg.norm(cube[:2])
        else:
            outward = np.array([1.0, 0.0])
        xy = min_radius * outward
    return np.array([xy[0], xy[1], height])


def level_rotation(forward, camera_x=None):
    """
    Camera axes (columns x, y, z) looking along `forward` with the image level, or
    None when that is straight up or down. Level can be upright or upside-down: with
    `camera_x`, the camera's current x axis, the nearer one is used.
    """
    right = np.cross(forward, UP)
    if np.linalg.norm(right) < MIN_HORIZONTAL_SIN:
        return None
    right /= np.linalg.norm(right)
    if camera_x is not None and np.dot(right, camera_x) < 0.0:
        right = -right
    return np.column_stack([right, np.cross(forward, right), forward])


def square_axes(forward, cube_axis):
    """
    The four directions in the image plane (perpendicular to `forward`) that line up
    with the cube's edges, as an array of rows; a cube looks the same turned by 90
    degrees. None if `cube_axis` is along `forward`.
    """
    forward = np.asarray(forward, dtype=float)
    axis = np.asarray(cube_axis, dtype=float)
    axis = axis - np.dot(axis, forward) * forward
    length = np.linalg.norm(axis)
    if length < 1e-6:
        return None
    axis /= length
    side = np.cross(forward, axis)
    return np.array([axis, side, -axis, -side])


def goal_pose(
    camera_position,
    cube_position,
    standoff=STANDOFF_M,
    camera_x=None,
    height=CAMERA_HEIGHT_M,
    min_radius=MIN_CAMERA_RADIUS_M,
    above=ABOVE_CUBE_M,
    cube_axis=None,
    max_reach=MAX_REACH_M,
):
    """
    The camera pose to chase the cube with: (position, rotation matrix whose columns
    are the camera x, y and z axes in the base frame), or None when no orientation can
    be worked out. With `cube_axis` (a horizontal direction along one of the cube's
    edges) the image lines up with the cube, using whichever edge is nearest the
    camera's current x axis `camera_x`. Otherwise the image is level. See
    chase_position and level_rotation.
    """
    position = chase_position(
        cube_position, standoff, height, min_radius, above, max_reach
    )
    to_cube = np.asarray(cube_position) - position
    forward = to_cube / np.linalg.norm(to_cube)
    if cube_axis is not None:
        axes = square_axes(forward, cube_axis)
        if axes is not None:
            if camera_x is None:
                x = axes[0]
            else:
                x = axes[int(np.argmax(axes @ np.asarray(camera_x)))]
            return position, np.column_stack([x, np.cross(forward, x), forward])
    rotation = level_rotation(forward, camera_x)
    if rotation is None:
        return None
    return position, rotation


def follow_twist(
    camera_position,
    camera_rotation,
    cube_position,
    standoff=STANDOFF_M,
    height=CAMERA_HEIGHT_M,
    keep_level=False,
    min_radius=MIN_CAMERA_RADIUS_M,
    above=ABOVE_CUBE_M,
    cube_axis=None,
    wrist_roll=None,
    max_reach=MAX_REACH_M,
):
    """
    Camera velocity to chase the cube: move to the chase position and aim at the cube.
    The aim only tilts the camera. With `cube_axis` (a horizontal direction along one
    of the cube's edges) the camera also rolls to line its image up with the cube,
    taking the nearest of the cube's four edges (it looks the same turned by 90
    degrees); `wrist_roll`, the last wrist joint's angle, makes it take the edge that
    keeps that joint nearest zero, since the joint is limited to +-90 degrees. Without
    `cube_axis`, roll is left alone, or with keep_level the image is turned level.
    Returns (linear, angular, distance_error, aim_error, roll_error) in the base
    frame; the twist is zero inside the dead-bands.
    """
    zero = np.zeros(3)
    to_cube = np.asarray(cube_position) - np.asarray(camera_position)
    distance = np.linalg.norm(to_cube)
    if distance < 1e-6:
        return zero, zero, 0.0, 0.0, 0.0
    forward = to_cube / distance
    axis_x = camera_rotation[:, 0]
    axis_z = camera_rotation[:, 2]

    # Move to the chase position, which follows the cube sideways and up and down.
    goal = chase_position(cube_position, standoff, height, min_radius, above, max_reach)
    position_error = goal - np.asarray(camera_position)
    distance_error = float(np.linalg.norm(position_error))
    linear = zero
    if distance_error > DISTANCE_DEADBAND_M:
        linear = clamp_norm(LINEAR_GAIN * position_error, MAX_LINEAR_MPS)

    # Turn the lens axis onto the line of sight.
    aim_axis = np.cross(axis_z, forward)
    aim_error = float(np.arcsin(min(1.0, np.linalg.norm(aim_axis))))
    if np.dot(axis_z, forward) < 0.0:
        aim_error = float(np.pi - aim_error)
    angular = zero
    if aim_error > AIM_DEADBAND_RAD:
        angular = ANGULAR_GAIN * aim_axis

    # Turn about the line of sight until the image's x axis is horizontal.
    roll_error = 0.0
    level_x = np.cross(forward, UP)
    horizontal = np.linalg.norm(level_x)
    if keep_level and horizontal >= MIN_HORIZONTAL_SIN:
        level_x /= horizontal
        # Near vertical, "level" swings around fast as the aim wobbles, so the roll
        # correction fades out between MIN_HORIZONTAL_SIN and FULL_ROLL_SIN.
        weight = min(
            1.0,
            (horizontal - MIN_HORIZONTAL_SIN) / (FULL_ROLL_SIN - MIN_HORIZONTAL_SIN),
        )
        roll_error = float(
            np.arctan2(
                np.dot(np.cross(axis_x, level_x), forward), np.dot(axis_x, level_x)
            )
        )
        # Level works upright or upside-down. Take the nearer, so the wrist never
        # has to roll more than 90 degrees (wrist_3 is limited to +-90 degrees).
        if roll_error > np.pi / 2:
            roll_error -= np.pi
        elif roll_error < -np.pi / 2:
            roll_error += np.pi
        if abs(roll_error) > ROLL_DEADBAND_RAD:
            angular = angular + weight * ANGULAR_GAIN * roll_error * forward

    # Roll the image into line with the cube. Only meaningful looking mostly down.
    if cube_axis is not None:
        looking_down = min(1.0, max(0.0, (-forward[2] - 0.5) / 0.4))
        axes = square_axes(forward, cube_axis)
        if looking_down > 0.0 and axes is not None:
            raw = float(
                np.arctan2(
                    np.dot(np.cross(axis_x, axes[0]), forward), np.dot(axis_x, axes[0])
                )
            )
            # Fold into +-45 degrees, then consider the other three edges too.
            base = (raw + np.pi / 4) % (np.pi / 2) - np.pi / 4
            wrist = 0.0 if wrist_roll is None else wrist_roll
            options = [base + k * np.pi / 2 for k in range(-3, 4)]
            roll_error = float(min(options, key=lambda r: abs(wrist + r)))
            if abs(roll_error) > ROLL_DEADBAND_RAD:
                angular = angular + looking_down * ANGULAR_GAIN * roll_error * forward

    angular = clamp_norm(angular, MAX_ANGULAR_RPS)
    return linear, angular, distance_error, aim_error, roll_error


def horizontal_axis(rotation):
    """
    A horizontal unit vector along one of the cube's edges, from the rotation of a tag
    on any face (columns are the tag's x, y and z axes in the base frame): the
    horizontal part of its x axis, or of its y axis for a tag whose x axis points up
    or down. Both lie along the cube's edges, so this is the cube's yaw, up to 90
    degrees. None if neither axis is horizontal enough.
    """
    rotation = np.asarray(rotation, dtype=float)
    for column in (0, 1):
        horizontal = rotation[:2, column]
        length = np.linalg.norm(horizontal)
        if length > 0.5:
            return np.array([horizontal[0] / length, horizontal[1] / length, 0.0])
    return None


def fourfold(axis):
    """An edge direction as a smoothable 2-vector that repeats every 90 degrees."""
    angle = np.arctan2(axis[1], axis[0])
    return np.array([np.cos(4.0 * angle), np.sin(4.0 * angle)])


def axis_from_fourfold(vector):
    """The inverse of fourfold (one of the four equivalent edge directions)."""
    angle = np.arctan2(vector[1], vector[0]) / 4.0
    return np.array([np.cos(angle), np.sin(angle), 0.0])


def quat_to_matrix(x, y, z, w):
    """Rotation matrix from a unit quaternion."""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(m):
    """Unit quaternion (x, y, z, w) from a rotation matrix."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        return (
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            s / 4.0,
        )
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4.0
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return (q[0], q[1], q[2], (m[k, j] - m[j, k]) / s)
