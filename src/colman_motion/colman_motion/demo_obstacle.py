import threading

import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor

from colman_motion.arm_control import ArmControl
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager

EE_DOWN = (1.0, 0.0, 0.0, 0.0)

SIDE_A = (0.25, -0.20, 0.10)
SIDE_B = (0.25, 0.10, 0.10)

# A block between A and B that is taller than the tool tip, so the straight
# move is blocked and the planner has to go over or around it.
BLOCK_DIMS = [0.06, 0.06, 0.15]
BLOCK_POS = (0.25, 0.0, 0.075)

CROSSINGS = 3


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.demo_obstacle")
    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")

    arm = ArmControl(ur)
    scene = SceneManager(ur)

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(scene)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        scene.add_box("table", "base_link", [2.0, 2.0, 0.01], (0.0, 0.0, -0.01))

        params = PlanRequestParameters(ur, "ompl")
        params.max_velocity_scaling_factor = 0.5
        params.max_acceleration_scaling_factor = 0.5

        logger.info("Moving to side A, no obstacle yet (expect Pilz)")
        if not arm.go_to_pose(*SIDE_A, *EE_DOWN, params):
            logger.error("Could not reach side A")
            return

        scene.add_box("block", "base_link", BLOCK_DIMS, BLOCK_POS)
        for i in range(CROSSINGS):
            for name, target in (("B", SIDE_B), ("A", SIDE_A)):
                logger.info(f"Crossing {i + 1}: moving to side {name} past the block")
                if not arm.go_to_pose(*target, *EE_DOWN, params):
                    logger.error(f"Failed to reach side {name}")
                    return
        logger.info("Obstacle test done")
    finally:
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
