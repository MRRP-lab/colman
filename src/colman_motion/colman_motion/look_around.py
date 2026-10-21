import threading
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from colman_motion.arm_control import ArmControl
from colman_motion.robot_ready import wait_for_robot

PAN_OFFSETS = (0.35, -0.35, 0.0)

DWELL_S = 3.0


class Seen(Node):
    def __init__(self):
        super().__init__("seen_tags")
        self.ids = set()
        self.create_subscription(AprilTagDetectionArray, "/detections", self.cb, 1)

    def cb(self, msg):
        self.ids.update(d.id for d in msg.detections)


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.look_around")
    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")
    arm = ArmControl(ur)
    seen = Seen()

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(seen)
    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        params = PlanRequestParameters(ur, "ompl")
        params.max_velocity_scaling_factor = 0.2
        params.max_acceleration_scaling_factor = 0.2

        with ur.get_planning_scene_monitor().read_only() as ps:
            home = dict(ps.current_state.joint_positions)
        for offset in PAN_OFFSETS:
            joints = dict(home)
            joints["shoulder_pan_joint"] = home["shoulder_pan_joint"] + offset
            if not arm.go_to_joint_pose(joints, params):
                logger.warn(f"Could not pan {offset}")
                continue
            seen.ids.clear()
            time.sleep(DWELL_S)
            logger.info(f"LOOK pan {offset:+.2f}: tags {sorted(seen.ids)}")
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
