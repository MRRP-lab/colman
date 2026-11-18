"""Relay /joint_states to /ms/joint_states so a test can drop them on demand.

The motion server is pointed at /ms/joint_states. Setting this node's `paused`
parameter to true stops the relay, which looks to the motion server like a robot
that has stopped reporting, while the driver itself keeps running.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState


class Relay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")
        self.declare_parameter("paused", False)
        self.pub = self.create_publisher(
            JointState,
            "/ms/joint_states",
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            JointState, "/joint_states", self.relay, qos_profile_sensor_data
        )

    def relay(self, msg):
        if not self.get_parameter("paused").value:
            self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(Relay())


if __name__ == "__main__":
    main()
