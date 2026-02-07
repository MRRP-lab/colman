import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

class StereoCameraInfoRepublisher(Node):
    """
    Reads a camera info message and adds the baseline distance between left and right sensors.
    """
    def __init__(self):
        super().__init__("stereo_camera_info_republisher")

        self.declare_parameter("baseline_m", 0.075)
        self.declare_parameter("left_camera_info_in", "/oakd_pro/left/camera_info")
        self.declare_parameter("right_camera_info_in", "/oakd_pro/right/camera_info")
        self.declare_parameter("left_camera_info_out", "/oakd_pro/left/camera_info_calibrated")
        self.declare_parameter("right_camera_info_out", "/oakd_pro/right/camera_info_calibrated")

        self.baseline_m = float(self.get_parameter("baseline_m").value)
        left_in = str(self.get_parameter("left_camera_info_in").value)
        right_in = str(self.get_parameter("right_camera_info_in").value)
        left_out = str(self.get_parameter("left_camera_info_out").value)
        right_out = str(self.get_parameter("right_camera_info_out").value)

        self.left_pub = self.create_publisher(CameraInfo, left_out, qos_profile_sensor_data)
        self.right_pub = self.create_publisher(CameraInfo, right_out, qos_profile_sensor_data)

        self.create_subscription(CameraInfo, left_in, self.left_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, right_in, self.right_cb, qos_profile_sensor_data)

    def copy_msg(self, msg: CameraInfo) -> CameraInfo:
        """
        :param msg: Incoming camera calibration message.
        :type msg: CameraInfo
        :return: Copied and normalized camera calibration message.
        :rtype: CameraInfo

        Copy a camera info message to edit and send as a stereo camera info message.
        """
        out = CameraInfo()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.distortion_model = msg.distortion_model
        out.d = list(msg.d)
        out.k = list(msg.k)
        out.r = list(msg.r)
        out.p = list(msg.p)
        out.binning_x = msg.binning_x
        out.binning_y = msg.binning_y
        out.roi = msg.roi
        if out.roi.width == 0:
            out.roi.width = out.width
        if out.roi.height == 0:
            out.roi.height = out.height
        out.roi.x_offset = 0
        out.roi.y_offset = 0
        return out

    def left_cb(self, msg: CameraInfo):
        """
        :param msg: Incoming left camera calibration message.
        :type msg: CameraInfo

        Validates left camera info and publishes.
        """
        out = self.copy_msg(msg)
        # left offset should be 0
        if len(out.p) >= 4:
            out.p[3] = 0.0
        self.left_pub.publish(out)

    def right_cb(self, msg: CameraInfo):
        """
        :param msg: Incoming right camera calibration message.
        :type msg: CameraInfo

        Validates right camera info and publishes.
        """
        out = self.copy_msg(msg)
        # get focal length
        fx = out.k[0] if len(out.k) > 0 and out.k[0] > 0.0 else (out.p[0] if len(out.p) > 0 else 0.0)
        # validate projection matrix and fx
        if len(out.p) >= 4 and fx > 0.0:
            out.p[3] = -fx * self.baseline_m
        self.right_pub.publish(out)


def main():
    rclpy.init()
    node = StereoCameraInfoRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
