import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from colman_perception.camera_names import device_info

CONTINUOUS_AF = dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO

# Cameras whose focus is locked at this lens position (0 to 255) instead of running
# autofocus. The overhead camera works out a tag's height from how big the tag looks,
# and continuous autofocus changes the image scale, so every tag's height jumped by
# centimetres together whenever a hand moved over the table. 118 is where its
# autofocus settles on the table.
LOCKED_LENS_POSITION = {"overhead": 118}

# How often to retry while no camera is connected.
RECONNECT_S = 2.0

# A camera that stops delivering frames this long is treated as unplugged. The camera
# can drop off USB mid-run, and without this the node keeps running and publishes
# nothing.
STALL_S = 3.0

# Topic prefix, frame, image size and frame rate for each named camera (names are in
# config/cameras.yaml). The wrist camera keeps the names the rest of the stack already
# uses. The overhead camera is 1.83 m from the table, where a 39.2 mm tag is only ~30 px
# wide at 1080p, so it runs at 4K. A 4K BGR frame is ~25 MB, so the rate is kept low to
# stay inside USB3 bandwidth; the cubes are static.
CAMERAS = {
    "wrist": (
        "/oakd_pro",
        "oakd_pro_camera_rgb_optical_frame_wrist",
        (1280, 720),
        15.0,
    ),
    "overhead": ("/overhead", "overhead_camera_optical_frame", (3840, 2160), 5.0),
}


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.declare_parameter("camera", "wrist")  # a name from config/cameras.yaml
        self.camera = self.get_parameter("camera").value
        if self.camera not in CAMERAS:
            raise ValueError(f"no topics set up for camera '{self.camera}'")
        prefix, self.frame_id, self.size, self.fps = CAMERAS[self.camera]

        self.image_pub = self.create_publisher(
            Image, f"{prefix}/rgb/image", qos_profile_sensor_data
        )
        self.info_pub = self.create_publisher(
            CameraInfo, f"{prefix}/rgb/camera_info", qos_profile_sensor_data
        )
        self.bridge = CvBridge()
        self.create_service(Trigger, f"{prefix}/refocus", self.on_refocus)

        # Frame metadata for downstream nodes
        self.info_msg = CameraInfo()
        self.info_msg.header.frame_id = self.frame_id
        self.info_msg.width, self.info_msg.height = self.size
        self.info_msg.distortion_model = "plumb_bob"
        self.info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.info_msg.r = np.eye(3).flatten().tolist()

        self.device = None
        self.pipeline = None
        self.queue = None
        self.control_queue = None
        self.last_meta = None
        self.last_frame = self.get_clock().now()
        self._info_ready = False

        self.connect()
        self.create_timer(1.0 / 30.0, self.timer_callback)
        self.create_timer(RECONNECT_S, self.reconnect_callback)

    def open_camera(self):
        """Open the named camera and start streaming.

        Returns (device, pipeline, frame queue, control queue). Raises RuntimeError if
        the camera is missing or busy.
        """
        device = dai.Device(device_info(self.camera))
        pipeline = dai.Pipeline(device)
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        # Autofocus and autoexposure are set explicitly, so they never depend on the
        # camera's defaults. Both keep adjusting as the arm moves and the view changes.
        if self.camera in LOCKED_LENS_POSITION:
            cam.initialControl.setManualFocus(LOCKED_LENS_POSITION[self.camera])
        else:
            cam.initialControl.setAutoFocusMode(CONTINUOUS_AF)
        cam.initialControl.setAutoExposureEnable()
        control_queue = cam.inputControl.createInputQueue()
        out = cam.requestOutput(self.size, enableUndistortion=True, fps=self.fps)
        queue = out.createOutputQueue(maxSize=4, blocking=False)
        pipeline.start()
        return device, pipeline, queue, control_queue

    def connect(self):
        try:
            self.device, self.pipeline, self.queue, self.control_queue = (
                self.open_camera()
            )
        except Exception as e:  # noqa: BLE001 - whatever an unplugged camera raises
            self.get_logger().warn(
                f"No {self.camera} camera yet ({e}), retrying",
                throttle_duration_sec=30.0,
            )
            self.disconnect()
            return
        self._info_ready = False
        self.last_frame = self.get_clock().now()
        self.get_logger().info(f"publishing images from the {self.camera} camera")

    def disconnect(self):
        """Close whatever is open and forget it, so the next retry starts clean."""
        pipeline, device = self.pipeline, self.device
        self.queue = None
        self.control_queue = None
        self.pipeline = None
        self.device = None
        for closer in (
            getattr(pipeline, "stop", None),
            getattr(device, "close", None),
        ):
            try:
                if closer is not None:
                    closer()
            except Exception:  # noqa: BLE001 - the device may already be gone
                pass

    def reconnect_callback(self):
        if self.device is None:
            self.connect()

    def on_refocus(self, request, response):
        """Start a fresh autofocus scan and re-run autoexposure.

        Called when the arm arrives at a new pose and is about to look for a tag, so the
        image settles in a moment instead of after the next slow drift.
        """
        if self.control_queue is None:
            response.success = False
            response.message = f"{self.camera} camera is not connected"
            return response
        ctrl = dai.CameraControl()
        if self.camera in LOCKED_LENS_POSITION:
            ctrl.setManualFocus(LOCKED_LENS_POSITION[self.camera])
        else:
            ctrl.setAutoFocusMode(CONTINUOUS_AF)
            ctrl.setAutoFocusTrigger()
        ctrl.setAutoExposureEnable()
        try:
            self.control_queue.send(ctrl)
        except RuntimeError as e:
            response.success = False
            response.message = f"refocus failed: {e}"
            return response
        response.success = True
        response.message = f"refocus requested; last frame {self.last_meta}"
        return response

    def timer_callback(self):
        if self.device is None:
            return
        try:
            if self.device.isClosed():
                raise RuntimeError("device closed")
            packet = self.queue.tryGet()
        except Exception as e:  # noqa: BLE001 - an unplugged camera raises many things
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
        self.last_meta = (
            f"lens {packet.getLensPosition()}, "
            f"exposure {packet.getExposureTime().total_seconds() * 1e6:.0f} us, "
            f"iso {packet.getSensitivity()}"
        )
        stamp = now.to_msg()

        if not self._info_ready:
            k = np.array(packet.getTransformation().getIntrinsicMatrix())
            self.info_msg.k = k.flatten().tolist()
            self.info_msg.p = np.hstack([k, np.zeros((3, 1))]).flatten().tolist()
            self._info_ready = True

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

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
        node.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
