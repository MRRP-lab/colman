import threading

import time
import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor

from colman_motion.arm_control import ArmControl
from colman_motion.scene_manager import SceneManager
from colman_motion.tag_lookup import TagLookup
from colman_motion.vacuum_control import VacuumControl

APPROACH_OFFSET = 0.1

EE_OFFSET = 0.01

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
        fast.max_velocity_scaling_factor = 0.7
        fast.max_acceleration_scaling_factor = 0.7

        slow = PlanRequestParameters(ur, "ompl")
        slow.max_velocity_scaling_factor = 0.3
        slow.max_acceleration_scaling_factor = 0.3

        # tool0 up Translation: [0.086, 0.129, 0.701]

        stop_event = arm.stop_event

        vacuum.release()

        tag_pattern = ["tag_1", "tag_3"]


        while not stop_event.is_set():
            old = None
            for tag_name in tag_pattern:
                tag_frame = tag_name
                if not arm.go_to_configured_pose("Look3", fast):
                    logger.warn("Stopping")
                    return
                tag_pose = None
                while tag_pose is None:
                    tag_pose = tag.get_tag_pose(tag_frame)

                tag_translation = tag_pose.translation
                pick_translation = (tag_translation.x, tag_translation.y, tag_translation.z - EE_OFFSET)
                approach_translation = (
                    tag_translation.x,
                    tag_translation.y,
                    tag_translation.z + APPROACH_OFFSET,
                )

                if not arm.go_to_pose(*approach_translation, *EE_DOWN, fast):
                    logger.warn("Stopping")
                    return

                if not arm.go_to_pose(*pick_translation, *EE_DOWN, slow):
                    logger.warn("Stopping")
                    return

                vacuum.grasp()

                if not arm.go_to_pose(*approach_translation, *EE_DOWN, fast):
                    logger.warn("Stopping")
                    return

                if old is None:
                    drop_off = DROP_OFF
                else:
                    tag_frame = old

                    if not arm.go_to_configured_pose("Look3", fast):
                        logger.warn("Stopping")
                        return

                    drop_pose = None
                    while drop_pose is None:
                        drop_pose = tag.get_tag_pose(tag_frame)

                    drop_off = (
                        drop_pose.translation.x,
                        drop_pose.translation.y,
                        drop_pose.translation.z + 0.05,
                    )

                drop_off_approach = (
                    drop_off[0],
                    drop_off[1],
                    drop_off[2] + APPROACH_OFFSET,
                )

                if not arm.go_to_pose(*drop_off_approach, *EE_DOWN, fast):
                    logger.warn("Stopping")
                    return
                
                if not arm.go_to_pose(*drop_off, *EE_DOWN, slow):
                    logger.warn("Stopping")
                    return
                
                vacuum.release()
                
                if not arm.go_to_pose(*drop_off_approach, *EE_DOWN, slow):
                    logger.warn("Stopping")
                    return
                old = tag_name
            logger.warn("\nreturning\n")    
            return
                

    finally:
        vacuum.release()
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
