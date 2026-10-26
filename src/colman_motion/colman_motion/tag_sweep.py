import os
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor

from colman_motion.arm_control import ArmControl
from colman_motion.click_move import (
    CAMERA_FRAME,
    TOOL_FRAME,
    Feed,
    level_camera_rotation,
)
from colman_motion.demo_camera import rotation_to_quaternion
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager

# How long the arm holds still looking at each tag, so tag_recorder can sample.
DWELL_S = 5.0

# The camera hovers this high, and as close to each corner as the arm reaches
# along the line out from the base, then looks at a point this far in from the
# corner toward the middle of the table, so both edges at the corner are in the
# picture along with plenty of the surface.
CAMERA_HEIGHT_M = 0.35
MAX_CAMERA_RADIUS_M = 0.38
STANDOFF_M = 0.2
LOOK_INWARD_M = 0.15


def load_tags():
    """
    The tag positions from the last recording, as {frame: (x, y, z)}.
    """
    path = os.path.join(
        get_package_share_directory("colman_perception"), "config", "tag_map.yaml"
    )
    with open(path) as f:
        tags = yaml.safe_load(f)["tags"]
    return {name: tuple(tag["position"]) for name, tag in tags.items()}


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.tag_sweep")
    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")

    arm = ArmControl(ur)
    scene = SceneManager(ur)
    feed = Feed()

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(scene)
    executor.add_node(feed)
    threading.Thread(target=executor.spin, daemon=True).start()

    params = PlanRequestParameters(ur, "ompl")
    params.max_velocity_scaling_factor = 0.3
    params.max_acceleration_scaling_factor = 0.3

    try:
        scene.add_box("table", "base_link", [2.0, 2.0, 0.01], (0.0, 0.0, -0.01))
        tags = load_tags()

        camera = tool = None
        while camera is None or tool is None:
            camera = feed.pose(CAMERA_FRAME)
            tool = feed.pose(TOOL_FRAME)
            time.sleep(0.2)
        # Where the camera sits on the hand, which never changes.
        hand_from_camera = camera[1].T @ tool[1]
        camera_in_hand = tool[1].T @ (camera[0] - tool[0])

        middle = np.mean([p[:2] for p in tags.values()], axis=0)
        for name, position in tags.items():
            corner = np.array(position[:2])
            inward = (middle - corner) / np.linalg.norm(middle - corner)
            outward = corner / np.linalg.norm(corner)
            radius = min(np.linalg.norm(corner) - STANDOFF_M, MAX_CAMERA_RADIUS_M)
            camera_at = np.array([*(outward * radius), CAMERA_HEIGHT_M])
            look_at = np.array([*(corner + LOOK_INWARD_M * inward), 0.0])
            direction = look_at - camera_at
            direction /= np.linalg.norm(direction)
            hand_rot = level_camera_rotation(direction) @ hand_from_camera
            hand = camera_at - hand_rot @ camera_in_hand
            quaternion = rotation_to_quaternion(*hand_rot.T)
            if not arm.go_to_pose(*hand, *quaternion, params):
                logger.warn(f"Could not look at {name}")
                continue
            logger.info(f"Looking at {name}")
            time.sleep(DWELL_S)
        logger.info("Sweep done")
    finally:
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
