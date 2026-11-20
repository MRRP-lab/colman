"""Publish AprilTag-style TF for tag_1..tag_8 so the motion server sees cubes.

Runs as a script from the integration test. The detector publishes a fresh stamp every
frame and TagLookup rejects a pose whose stamp has not changed, so the tags have to be
dynamic TF, not static.
"""

import rclpy
from colman_description.workcell import cube_top_z
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# tag_n -> (x, y) in base_link. Each tag is on the top of a 50 mm cube on the table.
TAG_XY = {
    1: (0.30, 0.15),
    2: (0.30, -0.10),
    3: (0.20, -0.20),
    4: (0.35, -0.25),
    5: (0.25, 0.25),
    6: (0.40, 0.05),
    7: (0.15, -0.15),
    8: (0.40, -0.10),
}
TAG_Z = cube_top_z(1)  # what world_state publishes for a cube on the table


class FakeTags(Node):
    def __init__(self):
        super().__init__("fake_tags")
        self.broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish)

    def publish(self):
        now = self.get_clock().now().to_msg()
        for number, (x, y) in TAG_XY.items():
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "base_link"
            tf.child_frame_id = f"tag_{number}"
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = TAG_Z
            tf.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    rclpy.spin(FakeTags())


if __name__ == "__main__":
    main()
