import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WINDOW = "depth (q quits)"

# Depth shown from near (red) to this far (blue), in millimeters.
MAX_MM = 1500.0


class DepthView(Node):
    def __init__(self):
        super().__init__("depth_view")
        self.bridge = CvBridge()
        self.create_subscription(
            Image, "/oakd_pro/depth/image", self.image_cb, qos_profile_sensor_data
        )

    def image_cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        scaled = 255 - np.clip(depth / MAX_MM * 255.0, 0, 255).astype(np.uint8)
        image = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
        image[depth == 0] = 0
        cv2.imshow(
            WINDOW, cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
        )


def main():
    rclpy.init()
    node = DepthView()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
