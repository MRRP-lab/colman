import threading

import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor

from colman_motion.arm_control import ArmControl
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager
from colman_motion.tag_lookup import BASE_FRAME, TagLookup

TOOL_FRAME = "vacuum_tip_link"

EE_UP = (0.0, 0.0, 0.0, 1.0)


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.hand_up")
    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")

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
        scene.add_box("table", "base_link", [2.0, 2.0, 0.01], (0.0, 0.0, -0.01))

        params = PlanRequestParameters(ur, "ompl")
        params.max_velocity_scaling_factor = 0.3
        params.max_acceleration_scaling_factor = 0.3

        tool = tag.lookup(TOOL_FRAME, BASE_FRAME)
        if tool is None:
            logger.error("Could not read the tool pose")
            return
        t = tool.transform.translation
        for dz in (0.0, 0.1, 0.2, 0.3):
            if arm.go_to_pose(t.x, t.y, t.z + dz, *EE_UP, params):
                return
            logger.warn(f"No plan to face the hand upwards at +{dz} m")
    finally:
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
