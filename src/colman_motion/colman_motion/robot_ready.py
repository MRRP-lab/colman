import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_STATE_TOPIC = "/joint_states"

REPORT_PERIOD = 5.0


def wait_for_robot(topic=JOINT_STATE_TOPIC, report_period=REPORT_PERIOD):
    """Block until a JointState arrives on `topic`.

    MoveItPy gives up after a short timeout with an unhelpful error if the
    robot isn't publishing yet, so call this before constructing it.
    """
    node = Node("wait_for_robot")
    logger = rclpy.logging.get_logger("wait_for_robot")
    received = []
    sub = node.create_subscription(JointState, topic, received.append, 10)

    logger.info(f"Waiting for robot: listening for joint states on '{topic}'...")
    waited = 0.0
    try:
        while not received:
            rclpy.spin_once(node, timeout_sec=report_period)
            if received:
                break
            waited += report_period
            publishers = node.count_publishers(topic)
            if publishers == 0:
                logger.warn(
                    f"Still waiting for robot after {waited:.0f}s: nothing is "
                    f"publishing on '{topic}'. Is the robot bringup (driver or "
                    "Gazebo + joint_state_broadcaster) running, with the same "
                    "ROS_DOMAIN_ID?"
                )
            else:
                logger.warn(
                    f"Still waiting for robot after {waited:.0f}s: {publishers} "
                    f"publisher(s) on '{topic}' but no messages yet. Is "
                    "joint_state_broadcaster active and the robot program running?"
                )
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()

    logger.info(f"Robot is online ({len(received[0].name)} joints reporting).")
