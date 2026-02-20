import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import numpy as np

class DisparityToDepth(Node):
    """
    Reads disparity info between left and right images,
    Calculates and outputs a depth image.
    """
    def __init__(self):
        super().__init__("disparity_to_depth")

        # params
        self.declare_parameter("disparity_topic", "/disparity")
        self.declare_parameter("depth_topic", "/oakd_pro/depth/image")
        self.declare_parameter("left_camera_info_topic", "/oakd_pro/left/camera_info")
        self.declare_parameter("depth_camera_info_topic", "/oakd_pro/depth/camera_info")
        self.declare_parameter("depth_frame_id", "oakd_pro_left_optical_frame_wrist")

        disparity_topic = self.get_parameter("disparity_topic").value
        depth_topic = self.get_parameter("depth_topic").value

        self.depth_frame_id = self.get_parameter("depth_frame_id").value
        left_camera_info_topic = self.get_parameter("left_camera_info_topic").value
        depth_camera_info_topic = self.get_parameter("depth_camera_info_topic").value

        self.bridge = CvBridge()

        self.latest_left_info = None

        # each time a disparity image arrives, run callback
        self.sub = self.create_subscription(
            DisparityImage,
            disparity_topic,
            self.cb,
            qos_profile_sensor_data,
        )

        # each time a camera info message arrives, run camera info callback
        self.cam_sub = self.create_subscription(
            CameraInfo,
            left_camera_info_topic,
            self.camera_info_cb,
            qos_profile_sensor_data,
        )

        # publishes depth image
        self.pub = self.create_publisher(
            Image,
            depth_topic,
            qos_profile_sensor_data,
        )

        # republishes camera info under depth topic
        self.cam_pub = self.create_publisher(
            CameraInfo,
            depth_camera_info_topic,
            qos_profile_sensor_data,
        )

        # logs
        self.get_logger().info(f"Subscribing: {disparity_topic}")
        self.get_logger().info(f"Using camera info: {left_camera_info_topic}")
        self.get_logger().info(f"Publishing : {depth_topic}")
        self.get_logger().info(f"Publishing : {depth_camera_info_topic}")

    def camera_info_cb(self, msg: CameraInfo):
        """
        :param msg: Incoming left camera calibration message.
        :type msg: CameraInfo

        Store camera info to be published alongside depth image
        """
        self.latest_left_info = msg

    def cb(self, msg: DisparityImage):
        """
        :param msg: Incoming stereo disparity frame.
        :type msg: DisparityImage

        Convert each disparity frame into a depth image.
        """

        # convert ros2 image into opencv format
        disp = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="32FC1").astype(np.float32)

        # flag invalid disparity
        if msg.t <= 0.0:
            self.get_logger().warn(
                f"Invalid disparity t={msg.t}.",
                throttle_duration_sec=5.0,
            )
            # set depth to all NaN
            depth = np.full_like(disp, np.nan, dtype=np.float32)
        else:
            # Convert disparity to depth
            depth = (msg.f * msg.t) / disp

        # set invalid values to NaN
        depth[~np.isfinite(depth)] = np.nan
        depth[disp <= 0.0] = np.nan

        # convert numpy format back to ros2 format and publish
        out = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding="32FC1")
        out.header = msg.image.header
        out.header.frame_id = self.depth_frame_id
        self.pub.publish(out)

        # prepare camera info for republish under depth frame topic
        if self.latest_left_info is not None:
            depth_info = CameraInfo()
            depth_info.header = out.header
            depth_info.height = self.latest_left_info.height
            depth_info.width = self.latest_left_info.width
            depth_info.distortion_model = self.latest_left_info.distortion_model
            depth_info.d = list(self.latest_left_info.d)
            depth_info.k = list(self.latest_left_info.k)
            depth_info.r = list(self.latest_left_info.r)
            depth_info.p = list(self.latest_left_info.p)
            depth_info.binning_x = self.latest_left_info.binning_x
            depth_info.binning_y = self.latest_left_info.binning_y
            depth_info.roi = self.latest_left_info.roi
            self.cam_pub.publish(depth_info)

def main():
    rclpy.init()
    node = DisparityToDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

