"""Print each visible tag's position in base_link (for checking a camera's accuracy)."""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PrintTags(Node):
    def __init__(self):
        super().__init__("print_tags")
        self.declare_parameter("base_frame", "base_link")
        self.base = self.get_parameter("base_frame").value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(2.0, self.report)

    def report(self):
        lines = []
        for i in range(1, 9):
            try:
                t = self.buffer.lookup_transform(self.base, f"tag_{i}", Time())
            except TransformException:
                continue
            p = t.transform.translation
            lines.append(f"tag_{i}: x={p.x:.4f} y={p.y:.4f} z={p.z:.4f} m")
        self.get_logger().info("\n" + ("\n".join(lines) or "no tags"))


def main():
    rclpy.init()
    node = PrintTags()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
