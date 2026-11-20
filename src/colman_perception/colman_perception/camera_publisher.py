from collections import deque, namedtuple
from concurrent.futures import Future, ThreadPoolExecutor

import cv2
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

# How often the camera's queue is read. A frame is read as soon as it is waiting, so
# this only has to be well above the frame rate.
POLL_HZ = 200.0

# Frames read in one poll before ROS callbacks (such as the refocus service) get a turn.
MAX_FRAMES_PER_POLL = 8

# Compressed frames being decoded at once. More than this means the computer cannot keep
# up, and new frames are dropped instead of building up delay.
MAX_IN_FLIGHT = 12

# The settings of each named camera (names are in config/cameras.yaml). The wrist camera
# keeps the topic names the rest of the stack already uses.
#
# size, fps:       the resolution and frame rate the camera is asked for.
# jpeg_quality:    0 sends raw frames. 1 to 100 has the camera compress each frame as
#                  MJPEG at that quality and this node decode it again.
# decode_threads:  threads that decode compressed frames.
#
# Both cameras are on USB 3 (5 Gbps), which carries about 270 MB/s, and were tested
# together with both detectors running:
#   wrist    1080p at 60 fps (the camera's maximum for 1080p) sends raw frames of
#            3.1 MB, 187 MB/s, with no dropped frames and about 50 ms of delay.
#   overhead 4K at 30 fps (the camera's maximum at 4K is 42). A 4K tag is about 62 px
#            wide at 1.83 m, so it needs 4K. Raw frames are 12.4 MB, which is 373 MB/s,
#            more than the link carries: 27% of frames were dropped. MJPEG at quality 90
#            is 1.6 MB per frame and had no drops when decoded on 3 threads; one thread
#            decodes too slowly (it dropped 8 to 24%). Lower qualities did no better,
#            because the limit is the decoding, not the link.
CameraSettings = namedtuple(
    "CameraSettings",
    ["prefix", "frame_id", "size", "fps", "jpeg_quality", "decode_threads"],
)
CAMERAS = {
    "wrist": CameraSettings(
        "/oakd_pro",
        "oakd_pro_camera_rgb_optical_frame_wrist",
        (1920, 1080),
        60.0,
        0,
        1,
    ),
    "overhead": CameraSettings(
        "/overhead", "overhead_camera_optical_frame", (3840, 2160), 30.0, 90, 3
    ),
}


def decode_jpeg(data):
    """A compressed frame as a BGR image, or None if it is damaged."""
    return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)


class CameraPublisher(Node):
    def __init__(self, **node_options):
        super().__init__("camera_publisher", **node_options)

        self.declare_parameter("camera", "wrist")  # a name from config/cameras.yaml
        self.camera = self.get_parameter("camera").value
        if self.camera not in CAMERAS:
            raise ValueError(f"no topics set up for camera '{self.camera}'")
        settings = CAMERAS[self.camera]
        prefix, self.frame_id, self.size = (
            settings.prefix,
            settings.frame_id,
            settings.size,
        )

        # These default to the camera's own settings in CAMERAS above.
        self.declare_parameter("fps", settings.fps)
        self.declare_parameter("jpeg_quality", settings.jpeg_quality)
        self.declare_parameter("decode_threads", settings.decode_threads)
        self.fps = float(self.get_parameter("fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.decode_threads = int(self.get_parameter("decode_threads").value)
        if self.fps <= 0:
            raise ValueError(f"fps must be above 0, got {self.fps}")
        if not 0 <= self.jpeg_quality <= 100:
            raise ValueError(f"jpeg_quality must be 0 to 100, got {self.jpeg_quality}")
        if self.decode_threads < 1:
            raise ValueError(
                f"decode_threads must be 1 or more, got {self.decode_threads}"
            )
        self.pool = (
            ThreadPoolExecutor(self.decode_threads) if self.jpeg_quality else None
        )
        # Frames waiting to be published, oldest first: (arrival time, a decoded image
        # or a future that will hold one). They go out in arrival order.
        self.pending = deque()

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
        self.create_timer(1.0 / POLL_HZ, self.timer_callback)
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
        if self.jpeg_quality == 0:
            out = cam.requestOutput(self.size, enableUndistortion=True, fps=self.fps)
        else:
            # The camera's encoder takes NV12 and compresses each frame on its own.
            nv12 = cam.requestOutput(
                self.size,
                type=dai.ImgFrame.Type.NV12,
                enableUndistortion=True,
                fps=self.fps,
            )
            out = (
                pipeline.create(dai.node.VideoEncoder)
                .build(
                    nv12,
                    frameRate=self.fps,
                    profile=dai.VideoEncoderProperties.Profile.MJPEG,
                    quality=self.jpeg_quality,
                )
                .out
            )
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
        self.pending.clear()
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
            packets = []
            while len(packets) < MAX_FRAMES_PER_POLL:
                packet = self.queue.tryGet()
                if packet is None:
                    break
                packets.append(packet)
        except Exception as e:  # noqa: BLE001 - an unplugged camera raises many things
            self.get_logger().warn(f"Camera lost ({e}), reconnecting")
            self.disconnect()
            return

        now = self.get_clock().now()
        if not packets:
            if (now - self.last_frame).nanoseconds > STALL_S * 1e9:
                self.get_logger().warn("Camera stalled, reconnecting")
                self.disconnect()
            else:
                self.publish_ready()
            return
        self.last_frame = now
        for packet in packets:
            self.accept(packet)
        self.publish_ready()

    def accept(self, packet):
        """Take one frame from the camera: use it now, or give it to a decode thread."""
        stamp = self.get_clock().now().to_msg()
        self.last_meta = (
            f"lens {packet.getLensPosition()}, "
            f"exposure {packet.getExposureTime().total_seconds() * 1e6:.0f} us, "
            f"iso {packet.getSensitivity()}"
        )
        if not self._info_ready:
            k = np.array(packet.getTransformation().getIntrinsicMatrix())
            self.info_msg.k = k.flatten().tolist()
            self.info_msg.p = np.hstack([k, np.zeros((3, 1))]).flatten().tolist()
            self._info_ready = True

        if self.pool is None:
            self.pending.append((stamp, packet.getCvFrame()))
        elif len(self.pending) >= MAX_IN_FLIGHT:
            self.get_logger().warn(
                "Decoding cannot keep up, dropping frames",
                throttle_duration_sec=10.0,
            )
        else:
            future = self.pool.submit(decode_jpeg, bytes(packet.getData()))
            self.pending.append((stamp, future))

    def publish_ready(self):
        """Publish the oldest frames that are ready, never ahead of an earlier one."""
        while self.pending:
            stamp, item = self.pending[0]
            if isinstance(item, Future):
                if not item.done():
                    return
                try:
                    frame = item.result()
                except Exception:  # noqa: BLE001 - a bad frame must not stop the stream
                    frame = None
            else:
                frame = item
            self.pending.popleft()
            if frame is None:
                self.get_logger().warn(
                    "Dropped a frame that would not decode", throttle_duration_sec=10.0
                )
                continue

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
        if node.pool is not None:
            node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
