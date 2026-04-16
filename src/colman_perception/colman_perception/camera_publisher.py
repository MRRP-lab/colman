import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

FRAME_ID = "oakd_pro_camera_rgb_optical_frame_wrist"
WIDTH = 1280
HEIGHT = 720


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.image_pub = self.create_publisher(
            Image, "/oakd_pro/rgb/image", qos_profile_sensor_data
        )
        self.info_pub = self.create_publisher(
            CameraInfo, "/oakd_pro/rgb/camera_info", qos_profile_sensor_data
        )
        self.bridge = CvBridge()

        self.device = dai.Device()
        self.pipeline = dai.Pipeline(self.device)
        pipeline = self.pipeline
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        out = cam.requestOutput((WIDTH, HEIGHT), enableUndistortion=True, fps=15.0)
        self.queue = out.createOutputQueue(maxSize=4, blocking=False)

        calib = self.device.getCalibration()
        k = np.array(
            calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, WIDTH, HEIGHT)
        )

        # Frame metadata for downstream nodes
        self.info_msg = CameraInfo()
        self.info_msg.header.frame_id = FRAME_ID
        self.info_msg.width = WIDTH
        self.info_msg.height = HEIGHT
        self.info_msg.distortion_model = "plumb_bob"
        self.info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.info_msg.k = k.flatten().tolist()
        self.info_msg.r = np.eye(3).flatten().tolist()
        self.info_msg.p = np.hstack([k, np.zeros((3, 1))]).flatten().tolist()

        pipeline.start()
        self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("publishing images")

    def timer_callback(self):
        packet = self.queue.tryGet()
        if packet is None:
            return
        frame = packet.getCvFrame()
        stamp = self.get_clock().now().to_msg()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_ID

        self.info_msg.header.stamp = stamp

        self.image_pub.publish(msg)
        self.info_pub.publish(self.info_msg)


def main():
    rclpy.init()
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.device.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
