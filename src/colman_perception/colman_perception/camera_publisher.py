import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

FRAME_ID = "oakd_pro_camera_rgb_optical_frame_wrist"
WIDTH = 1920
HEIGHT = 1080

# Depth is computed from the stereo pair, aligned to the color camera and
# scaled down from there.
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 360

IR_DOT_INTENSITY = 0.6

# How often to retry while no camera is connected.
RECONNECT_S = 2.0

# A camera that stops delivering frames this long is treated as unplugged.
STALL_S = 3.0


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.image_pub = self.create_publisher(
            Image, "/oakd_pro/rgb/image", qos_profile_sensor_data
        )
        self.info_pub = self.create_publisher(
            CameraInfo, "/oakd_pro/rgb/camera_info", qos_profile_sensor_data
        )
        self.depth_pub = self.create_publisher(
            Image, "/oakd_pro/depth/image", qos_profile_sensor_data
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, "/oakd_pro/depth/camera_info", qos_profile_sensor_data
        )
        self.bridge = CvBridge()

        self.device = None
        self.pipeline = None
        self.queue = None
        self.depth_queue = None
        self.last_frame = self.get_clock().now()
        self._info_ready = False
        self._depth_info_ready = False

        # Frame metadata for downstream nodes
        self.info_msg = CameraInfo()
        self.info_msg.header.frame_id = FRAME_ID
        self.info_msg.width = WIDTH
        self.info_msg.height = HEIGHT
        self.info_msg.distortion_model = "plumb_bob"
        self.info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.info_msg.r = np.eye(3).flatten().tolist()

        self.depth_info_msg = CameraInfo()
        self.depth_info_msg.header.frame_id = FRAME_ID
        self.depth_info_msg.width = DEPTH_WIDTH
        self.depth_info_msg.height = DEPTH_HEIGHT
        self.depth_info_msg.distortion_model = "plumb_bob"
        self.depth_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.depth_info_msg.r = np.eye(3).flatten().tolist()

        self.create_timer(1.0 / 30.0, self.timer_callback)
        self.create_timer(RECONNECT_S, self.reconnect_callback)

    def connect(self):
        try:
            self.device = dai.Device()
            # The dots give the stereo cameras something to match on the plain,
            # black tabletop.
            self.device.setIrLaserDotProjectorIntensity(IR_DOT_INTENSITY)
            self.pipeline = dai.Pipeline(self.device)
            cam = self.pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A
            )
            out = cam.requestOutput((WIDTH, HEIGHT), enableUndistortion=True, fps=15.0)
            self.queue = out.createOutputQueue(maxSize=4, blocking=False)

            stereo = self.pipeline.create(dai.node.StereoDepth).build(
                autoCreateCameras=True
            )
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            stereo.setOutputSize(DEPTH_WIDTH, DEPTH_HEIGHT)
            stereo.setExtendedDisparity(True)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)
            self.depth_queue = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
            self.pipeline.start()
        except RuntimeError as e:
            self.get_logger().warn(
                f"No camera yet ({e}), retrying", throttle_duration_sec=30.0
            )
            self.disconnect()
            return
        self._info_ready = False
        self._depth_info_ready = False
        self.last_frame = self.get_clock().now()
        self.get_logger().info("publishing images")

    def disconnect(self):
        self.queue = None
        self.depth_queue = None
        for closer in (
            getattr(self.pipeline, "stop", None),
            getattr(self.device, "close", None),
        ):
            try:
                if closer is not None:
                    closer()
            except Exception:
                pass
        self.pipeline = None
        self.device = None

    def reconnect_callback(self):
        if self.device is None:
            self.connect()

    def timer_callback(self):
        if self.device is None:
            return
        try:
            if self.device.isClosed():
                raise RuntimeError("device closed")
            packet = self.queue.tryGet()
        except RuntimeError as e:
            self.get_logger().warn(f"Camera lost ({e}), reconnecting")
            self.disconnect()
            return

        now = self.get_clock().now()
        if packet is None:
            if (now - self.last_frame).nanoseconds > STALL_S * 1e9:
                self.get_logger().warn("Camera stalled, reconnecting")
                self.disconnect()
            return
        self.last_frame = now

        frame = packet.getCvFrame()
        stamp = now.to_msg()

        if not self._info_ready:
            k = np.array(packet.getTransformation().getIntrinsicMatrix())
            self.info_msg.k = k.flatten().tolist()
            self.info_msg.p = np.hstack([k, np.zeros((3, 1))]).flatten().tolist()
            self._info_ready = True

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_ID

        self.info_msg.header.stamp = stamp

        self.image_pub.publish(msg)
        self.info_pub.publish(self.info_msg)
        self.publish_depth(stamp)

    def publish_depth(self, stamp):
        packet = self.depth_queue.tryGet()
        if packet is None:
            return
        depth = packet.getFrame().astype(np.uint16)

        if not self._depth_info_ready and self._info_ready:
            # The depth is aligned to the color camera and scaled by the same
            # amount in both directions, so its intrinsics are the color
            # camera's, scaled down. The matrix the depth packet reports has a
            # wrong vertical focal length.
            # Pixel centers sit at half-integer coordinates, so the principal
            # point scales as (c + 0.5) / n - 0.5.
            k = np.array(self.info_msg.k).reshape(3, 3)
            scale = np.array([DEPTH_WIDTH / WIDTH, DEPTH_HEIGHT / HEIGHT])
            k[:2, :2] *= scale[:, None]
            k[:2, 2] = (k[:2, 2] + 0.5) * scale - 0.5
            self.depth_info_msg.k = k.flatten().tolist()
            self.depth_info_msg.p = np.hstack([k, np.zeros((3, 1))]).flatten().tolist()
            self.depth_info_msg.width = depth.shape[1]
            self.depth_info_msg.height = depth.shape[0]
            self._depth_info_ready = True
        if not self._depth_info_ready:
            return

        # 16UC1, millimeters, 0 where there's no depth.
        msg = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_ID
        self.depth_info_msg.header.stamp = stamp
        self.depth_pub.publish(msg)
        self.depth_info_pub.publish(self.depth_info_msg)


def main():
    rclpy.init()
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
