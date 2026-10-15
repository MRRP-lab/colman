import math
import threading

import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor

from colman_motion.arm_control import ArmControl
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager

EE_DOWN = (1.0, 0.0, 0.0, 0.0)

CIRCLE_CENTER = (0.25, 0.0, 0.2)

CIRCLE_RADIUS = 0.08

CIRCLE_POINTS = 12


def circle_waypoints():
    cx, cy, cz = CIRCLE_CENTER
    for i in range(CIRCLE_POINTS):
        angle = 2 * math.pi * i / CIRCLE_POINTS
        yield (
            cx + CIRCLE_RADIUS * math.cos(angle),
            cy + CIRCLE_RADIUS * math.sin(angle),
            cz,
        )


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.demo_circle")
    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")
    logger.info("MoveItPy instance created")

    arm = ArmControl(ur)
    scene = SceneManager(ur)

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(scene)

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
            for waypoint in circle_waypoints():
                if not arm.go_to_pose(*waypoint, *EE_DOWN, params):
                    logger.warn("Stopping")
                    return

    finally:
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
