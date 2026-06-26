import threading

import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor

from colman_motion.arm_control import ArmControl
from colman_motion.scene_manager import SceneManager
from colman_motion.tag_lookup import TagLookup
from colman_motion.vacuum_control import VacuumControl

APPROACH_OFFSET = 0.05

EE_DOWN = (1.0, 0.0, 0.0, 0.0)

DROP_OFF = (0.2, -0.2, 0.044)


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")
    ur = MoveItPy(node_name="moveit_py")
    logger.info("MoveItPy instance created")

    arm = ArmControl(ur)
    scene = SceneManager(ur)
    vacuum = VacuumControl()
    tag = TagLookup()

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(scene)
    executor.add_node(vacuum)
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

        fast = PlanRequestParameters(ur, "ompl")
        fast.max_velocity_scaling_factor = 0.3
        fast.max_acceleration_scaling_factor = 0.3

        slow = PlanRequestParameters(ur, "ompl")
        slow.max_velocity_scaling_factor = 0.1
        slow.max_acceleration_scaling_factor = 0.1

        # tool0 up Translation: [0.086, 0.129, 0.701]

        stop_event = arm.stop_event

        vacuum.release()

        while not stop_event.is_set():
            if not arm.go_to_configured_pose("Look4", slow):
                logger.warn("Stopping")
                return

            tag_pose = tag.get_tag_pose()

            if tag_pose is None:
                continue

            tag_translation = tag_pose.translation
            pick_translation = (tag_translation.x, tag_translation.y, tag_translation.z)
            approach_translation = (
                tag_translation.x,
                tag_translation.y,
                tag_translation.z + APPROACH_OFFSET,
            )

            if not arm.go_to_pose(*approach_translation, *EE_DOWN, slow):
                logger.warn("Stopping")
                return

            if not arm.go_to_pose(*pick_translation, *EE_DOWN, slow):
                logger.warn("Stopping")
                return

            vacuum.grasp()

            if not arm.go_to_pose(*approach_translation, *EE_DOWN, slow):
                logger.warn("Stopping")
                return

            drop_off_approach = (
                DROP_OFF[0],
                DROP_OFF[1],
                DROP_OFF[2] + APPROACH_OFFSET,
            )

            if not arm.go_to_pose(*drop_off_approach, *EE_DOWN, slow):
                logger.warn("Stopping")
                return

            if not arm.go_to_pose(*DROP_OFF, *EE_DOWN, slow):
                logger.warn("Stopping")
                return

            vacuum.release()

            if not arm.go_to_pose(*drop_off_approach, *EE_DOWN, slow):
                logger.warn("Stopping")
                return

    finally:
        vacuum.release()
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
