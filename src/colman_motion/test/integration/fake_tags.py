"""Publish fake AprilTag TF for tag_1..tag_8, the way the two real cameras would.

Runs as a script from the integration test.

- The overhead camera sees every tag all the time, but only to about 1 cm: it publishes
  ovh_tag_N with a small fixed error.
- The wrist camera is exact but only sees a tag when the tool is held over it: it
  publishes tag_N only while the tool tip is within WRIST_RADIUS_M of the tag
  horizontally and between WRIST_MIN_M and WRIST_MAX_M above it.

Each frame gets a new stamp every publish, because TagLookup rejects a pose whose stamp
has not changed.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

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
OVERHEAD_ERROR_M = (0.008, -0.006, 0.004)  # what "good to about 1 cm" looks like
TOOL_FRAME = "vacuum_tip_link"
WRIST_RADIUS_M = 0.08
WRIST_MIN_M = 0.10
WRIST_MAX_M = 0.50


class FakeTags(Node):
    def __init__(self):
        super().__init__("fake_tags")
        self.broadcaster = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(0.05, self.publish)

    def tool_position(self):
        try:
            tf = self.buffer.lookup_transform("base_link", TOOL_FRAME, Time())
        except TransformException:
            return None
        return tf.transform.translation

    def wrist_sees(self, tool, x, y):
        if tool is None:
            return False
        above = tool.z - TAG_Z
        near = abs(tool.x - x) < WRIST_RADIUS_M and abs(tool.y - y) < WRIST_RADIUS_M
        return near and WRIST_MIN_M < above < WRIST_MAX_M

    def send(self, stamp, child, x, y, z):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "base_link"
        tf.child_frame_id = child
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(tf)

    def publish(self):
        now = self.get_clock().now().to_msg()
        tool = self.tool_position()
        ex, ey, ez = OVERHEAD_ERROR_M
        for number, (x, y) in TAG_XY.items():
            self.send(now, f"ovh_tag_{number}", x + ex, y + ey, TAG_Z + ez)
            if self.wrist_sees(tool, x, y):
                self.send(now, f"tag_{number}", x, y, TAG_Z)


def main():
    rclpy.init()
    rclpy.spin(FakeTags())


if __name__ == "__main__":
    main()
