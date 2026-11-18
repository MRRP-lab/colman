"""Publish AprilTag-style TF for tag_1..tag_8 so the motion server sees cubes.

Runs as a script from the integration test. The detector publishes a fresh stamp every
frame and TagLookup rejects a pose whose stamp has not changed, so the tags have to be
dynamic TF, not static.

Tests steer it through this node's parameters:

  hidden     tag numbers that stop being published (the tag is "not visible")
  overrides  JSON {"2": [x, y, z, qx, qy, qz, qw]} replacing a tag's pose
"""

import json

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# tag_n -> (x, y) in base_link. The tag is on top of a 39 mm cube standing on the table.
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
TAG_Z = 0.039


class FakeTags(Node):
    def __init__(self):
        super().__init__("fake_tags")
        self.declare_parameter("hidden", [0])
        self.declare_parameter("overrides", "{}")
        self.broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish)

    def publish(self):
        hidden = set(self.get_parameter("hidden").value)
        overrides = json.loads(self.get_parameter("overrides").value)
        now = self.get_clock().now().to_msg()
        for number, (x, y) in TAG_XY.items():
            if number in hidden:
                continue
            pose = overrides.get(str(number), [x, y, TAG_Z, 0.0, 0.0, 0.0, 1.0])
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "base_link"
            tf.child_frame_id = f"tag_{number}"
            translation, rotation = tf.transform.translation, tf.transform.rotation
            translation.x, translation.y, translation.z = pose[:3]
            rotation.x, rotation.y, rotation.z, rotation.w = pose[3:]
            self.broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    rclpy.spin(FakeTags())


if __name__ == "__main__":
    main()
