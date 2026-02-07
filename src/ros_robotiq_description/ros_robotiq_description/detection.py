import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO

class DetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.bridge = CvBridge()
        self.pub_annotated = self.create_publisher(
            Image,
            "/yolo/annotated",
            10
        )
        self.latest_image = None
        self.latest_header = None

        self.create_subscription(Image, "/oakd_pro/rgb/image", self.image_callback, 10)
        self.model = YOLO("yolo11m-seg.pt")

        # 20hz
        self.create_timer(0.05, self.detect_timer_callback)
        self.get_logger().info("Detection Node Started.")

    def image_callback(self, msg: Image):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.latest_header = msg.header

    def detect_timer_callback(self):
        if self.latest_image is None:
            return

        frame = self.latest_image.copy()

        # Run model detection
        results = self.model(frame, verbose=False)

        # Draw detections for visualization
        annotated = results[0].plot()

        # publish
        msg_out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        msg_out.header = self.latest_header
        self.pub_annotated.publish(msg_out)

def main():
    rclpy.init()
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()