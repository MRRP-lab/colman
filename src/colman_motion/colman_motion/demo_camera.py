import math
import threading
import time

import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from colman_motion.arm_control import ArmControl
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager
from colman_motion.tag_lookup import BASE_FRAME, TAG_FRAME, TagLookup

TOOL_FRAME = "vacuum_tip_link"

UP = (0.0, 0.0, 1.0)

MAX_TAG_AGE_S = 1.0

MIN_DISTANCE_M = 0.05

# Only re-aim when the hand is off the cube by more than this.
AIM_TOLERANCE_RAD = math.radians(5.0)

POLL_S = 0.2


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalize(v):
    n = math.sqrt(dot(v, v))
    return tuple(x / n for x in v)


def look_at_rotation(direction):
    """
    Rotation matrix columns (x, y, z) whose z axis points along direction.
    """
    z = normalize(direction)
    ref = UP if abs(dot(z, UP)) < 0.99 else (1.0, 0.0, 0.0)
    x = normalize(cross(ref, z))
    y = cross(z, x)
    return x, y, z


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


def tool_z_axis(q):
    qx, qy, qz, qw = q
    return (
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )


def translation(transform):
    t = transform.transform.translation
    return (t.x, t.y, t.z)


def rotation(transform):
    r = transform.transform.rotation
    return (r.x, r.y, r.z, r.w)


def fresh_tag(tag):
    """
    Returns the cube's latest transform if the tag was seen recently.
    """
    transform = tag.lookup(TAG_FRAME, BASE_FRAME)
    if transform is None:
        return None
    age = (tag.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds
    if age > MAX_TAG_AGE_S * 1e9:
        return None
    return transform


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.demo_camera")
    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")
    logger.info("MoveItPy instance created")

    arm = ArmControl(ur)
    scene = SceneManager(ur)
    tag = TagLookup()

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(scene)
    executor.add_node(tag)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        scene.add_box(
            "table",
            "base_link",
            [2.0, 2.0, 0.01],
            (0.0, 0.0, -0.01),
        )

        params = PlanRequestParameters(ur, "ompl")
        params.max_velocity_scaling_factor = 0.3
        params.max_acceleration_scaling_factor = 0.3

        while not arm.stop_event.is_set():
            cube = fresh_tag(tag)
            tool = tag.lookup(TOOL_FRAME, BASE_FRAME)
            if cube is None or tool is None:
                time.sleep(POLL_S)
                continue

            position = translation(tool)
            direction = sub(translation(cube), position)
            if math.sqrt(dot(direction, direction)) < MIN_DISTANCE_M:
                time.sleep(POLL_S)
                continue

            target = rotation_to_quaternion(*look_at_rotation(direction))
            facing = dot(tool_z_axis(rotation(tool)), normalize(direction))
            aim_error = math.acos(max(-1.0, min(1.0, facing)))
            if aim_error < AIM_TOLERANCE_RAD:
                time.sleep(POLL_S)
                continue

            if not arm.go_to_pose(*position, *target, params):
                logger.warn("Stopping")
                return

    finally:
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
