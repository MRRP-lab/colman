import threading
import time

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

from colman_motion.arm_control import ArmControl
from colman_motion.demo_camera import rotation_to_quaternion
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager

BASE_FRAME = "base_link"
TOOL_FRAME = "vacuum_tip_link"
CAMERA_FRAME = "oakd_pro_camera_rgb_optical_frame_wrist"

# The hand stays here, only its rotation changes. Middle of the table, 30 cm up.
HAND_POSITION = (0.3, 0.0, 0.3)

# Height of the point the camera aims at, the top of a tag cube.
AIM_HEIGHT_M = 0.04

TABLE_WINDOW = "click to aim (q quits)"
CAMERA_WINDOW = "camera"

# Top-down view: a square of VIEW_SPAN_M around the base, +x up, +y left.
VIEW_SPAN_M = 1.2
VIEW_PX = 900
PX_PER_M = VIEW_PX / VIEW_SPAN_M
GRID_M = 0.1

# Maps (x, y, 1) in the base frame to view pixels.
WORLD_TO_VIEW = np.array(
    [
        [0.0, -PX_PER_M, VIEW_PX / 2],
        [-PX_PER_M, 0.0, VIEW_PX / 2],
        [0.0, 0.0, 1.0],
    ]
)

CAMERA_VIEW_WIDTH = 960


def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class Feed(Node):
    """
    Keeps the latest camera image and tag detections, and looks up frames.
    """

    def __init__(self):
        super().__init__("click_move_feed")
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.image = None
        self.detections = []
        self.create_subscription(
            Image, "/oakd_pro/rgb/image", self.image_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            AprilTagDetectionArray, "/detections", self.detections_cb, 1
        )

    def image_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.image = image

    def detections_cb(self, msg):
        self.detections = list(msg.detections)

    def snapshot(self):
        with self.lock:
            return None if self.image is None else self.image.copy()

    def pose(self, frame):
        """
        Returns the frame's (position, rotation matrix) in the base frame, or None.
        """
        try:
            transform = self.buffer.lookup_transform(BASE_FRAME, frame, Time())
        except TransformException:
            return None
        t = transform.transform.translation
        r = transform.transform.rotation
        return (
            np.array([t.x, t.y, t.z]),
            quaternion_to_matrix((r.x, r.y, r.z, r.w)),
        )

    def rotation(self, frame):
        """
        Returns the frame's rotation matrix in the base frame, or None.
        """
        pose = self.pose(frame)
        return None if pose is None else pose[1]


def level_camera_rotation(direction):
    """
    Camera rotation (columns x right, y down, z forward) looking along direction
    with the image's up side toward world up. Looking straight down, up is +x.
    """
    down = np.array([0.0, 0.0, -1.0])
    if abs(np.dot(direction, down)) > 0.99:
        down = np.array([-1.0, 0.0, 0.0])
    y = down - np.dot(down, direction) * direction
    y /= np.linalg.norm(y)
    x = np.cross(y, direction)
    return np.column_stack([x, y, direction])


def view_point(world):
    p = WORLD_TO_VIEW @ np.array([world[0], world[1], 1.0])
    return int(p[0]), int(p[1])


def view_to_world(u, v):
    x, y, w = np.linalg.inv(WORLD_TO_VIEW) @ np.array([u, v, 1.0])
    return float(x / w), float(y / w)


def tabletop():
    view = np.full((VIEW_PX, VIEW_PX, 3), (95, 110, 125), np.uint8)
    for i in range(-6, 7):
        along = i * GRID_M
        cv2.line(
            view,
            view_point((along, -0.6)),
            view_point((along, 0.6)),
            (130, 130, 130),
            1,
        )
        cv2.line(
            view,
            view_point((-0.6, along)),
            view_point((0.6, along)),
            (130, 130, 130),
            1,
        )
    for i in range(0, 7):
        cv2.putText(
            view,
            f"x={i * GRID_M:.1f}",
            view_point((i * GRID_M, -0.58)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (220, 220, 220),
            1,
        )
    cv2.drawMarker(view, view_point((0, 0)), (255, 255, 255), cv2.MARKER_DIAMOND, 24, 2)
    cv2.drawMarker(
        view,
        view_point(HAND_POSITION[:2]),
        (255, 128, 0),
        cv2.MARKER_TILTED_CROSS,
        30,
        3,
    )
    return view


def facing_point(camera_rot):
    """
    Where the camera's optical axis, from the hand, meets the aim height, or None.
    """
    axis = camera_rot[:, 2]
    if axis[2] > -0.05:
        return None
    reach = (AIM_HEIGHT_M - HAND_POSITION[2]) / axis[2]
    return HAND_POSITION[0] + reach * axis[0], HAND_POSITION[1] + reach * axis[1]


def draw_facing(background, camera_rot):
    view = background.copy()
    point = facing_point(camera_rot)
    if point is not None:
        cv2.line(view, view_point(HAND_POSITION[:2]), view_point(point), (0, 0, 255), 2)
        cv2.drawMarker(view, view_point(point), (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
    return view


def draw_tags(image, detections):
    for d in detections:
        pts = np.array([[c.x, c.y] for c in d.corners], np.int32)
        cv2.polylines(image, [pts], True, (0, 255, 0), 4)
        cv2.putText(
            image,
            f"tag {d.id}",
            tuple(pts[0]),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (0, 255, 0),
            4,
        )
    scale = CAMERA_VIEW_WIDTH / image.shape[1]
    return cv2.resize(image, None, fx=scale, fy=scale)


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.click_move")
    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")

    arm = ArmControl(ur)
    scene = SceneManager(ur)
    feed = Feed()

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(scene)
    executor.add_node(feed)
    threading.Thread(target=executor.spin, daemon=True).start()

    params = PlanRequestParameters(ur, "ompl")
    params.max_velocity_scaling_factor = 0.3
    params.max_acceleration_scaling_factor = 0.3

    try:
        scene.add_box("table", "base_link", [2.0, 2.0, 0.01], (0.0, 0.0, -0.01))

        # How the hand is turned relative to the camera, which never changes.
        camera_rot = tool_rot = None
        while camera_rot is None or tool_rot is None:
            camera_rot = feed.rotation(CAMERA_FRAME)
            tool_rot = feed.rotation(TOOL_FRAME)
            time.sleep(0.2)
        hand_from_camera = camera_rot.T @ tool_rot

        def aim(x, y):
            direction = np.array([x, y, AIM_HEIGHT_M]) - np.array(HAND_POSITION)
            direction /= np.linalg.norm(direction)
            hand_rot = level_camera_rotation(direction) @ hand_from_camera
            quaternion = rotation_to_quaternion(*hand_rot.T)
            if arm.go_to_pose(*HAND_POSITION, *quaternion, params):
                logger.info(f"aimed at x={x:.3f} y={y:.3f}")
            else:
                logger.warn(f"could not aim at x={x:.3f} y={y:.3f}")

        busy = threading.Lock()

        def aim_and_release(x, y):
            try:
                aim(x, y)
            finally:
                busy.release()

        def on_mouse(event, u, v, flags, param):
            if event != cv2.EVENT_LBUTTONDOWN or not busy.acquire(blocking=False):
                return
            threading.Thread(
                target=aim_and_release, args=view_to_world(u, v), daemon=True
            ).start()

        background = tabletop()
        cv2.namedWindow(TABLE_WINDOW)
        cv2.setMouseCallback(TABLE_WINDOW, on_mouse)
        cv2.imshow(TABLE_WINDOW, background)

        if busy.acquire(blocking=False):
            threading.Thread(
                target=aim_and_release, args=(HAND_POSITION[0], 0.0), daemon=True
            ).start()

        while rclpy.ok() and not arm.stop_event.is_set():
            camera_rot = feed.rotation(CAMERA_FRAME)
            if camera_rot is not None:
                cv2.imshow(TABLE_WINDOW, draw_facing(background, camera_rot))
            image = feed.snapshot()
            if image is not None:
                cv2.imshow(CAMERA_WINDOW, draw_tags(image, feed.detections))
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
