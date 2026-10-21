import math
import queue
import threading

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from colman_motion.arm_control import ArmControl
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager

BASE_FRAME = "base_link"

EE_DOWN = (1.0, 0.0, 0.0, 0.0)

HOVER_HEIGHT_M = 0.3

# Clicks outside this ring around the base are refused, the arm can't reach them.
MIN_REACH_M = 0.15
MAX_REACH_M = 0.55

TOOL_FRAME = "vacuum_tip_link"

WINDOW = "click to move (q quits)"

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


def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class CameraView(Node):
    """
    Keeps the latest camera frame, its intrinsics and the detected tags,
    and turns a clicked pixel into a point on the table.
    """

    def __init__(self):
        super().__init__("camera_view")
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.image = None
        self.stamp = None
        self.frame_id = None
        self.k = None
        self.detections = []
        self.create_subscription(
            Image, "/oakd_pro/rgb/image", self.image_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/oakd_pro/rgb/camera_info",
            self.info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AprilTagDetectionArray, "/detections", self.detections_cb, 1
        )

    def image_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.image = image
            self.stamp = msg.header.stamp
            self.frame_id = msg.header.frame_id

    def info_cb(self, msg):
        self.k = np.array(msg.k).reshape(3, 3)

    def detections_cb(self, msg):
        self.detections = list(msg.detections)

    def snapshot(self):
        with self.lock:
            if self.image is None:
                return None
            return self.image.copy(), self.stamp, self.frame_id

    def table_homography(self, stamp, frame_id):
        """
        Returns the matrix mapping table points (x, y, 1) in the base frame
        to camera pixels, or None.
        """
        if self.k is None:
            return None
        try:
            transform = self.buffer.lookup_transform(
                BASE_FRAME, frame_id, Time.from_msg(stamp), Duration(seconds=0.5)
            )
        except TransformException:
            return None
        t = transform.transform.translation
        r = transform.transform.rotation
        to_cam = quaternion_to_matrix((r.x, r.y, r.z, r.w)).T
        origin = to_cam @ np.array([t.x, t.y, t.z])
        table_to_cam = np.column_stack([to_cam[:, 0], to_cam[:, 1], -origin])
        return self.k @ table_to_cam

    def tool_xy(self):
        try:
            transform = self.buffer.lookup_transform(BASE_FRAME, TOOL_FRAME, Time())
        except TransformException:
            return None
        t = transform.transform.translation
        return t.x, t.y


def view_point(world):
    p = WORLD_TO_VIEW @ np.array([world[0], world[1], 1.0])
    return int(p[0]), int(p[1])


def view_to_world(u, v):
    x, y, w = np.linalg.inv(WORLD_TO_VIEW) @ np.array([u, v, 1.0])
    return float(x / w), float(y / w)


def topdown(image, homography):
    to_view = WORLD_TO_VIEW @ np.linalg.inv(homography)
    return cv2.warpPerspective(image, to_view, (VIEW_PX, VIEW_PX))


def draw(view, homography, detections, target, tool, status):
    reachable = np.zeros(view.shape[:2], np.uint8)
    cv2.circle(reachable, view_point((0, 0)), int(MAX_REACH_M * PX_PER_M), 255, -1)
    cv2.circle(reachable, view_point((0, 0)), int(MIN_REACH_M * PX_PER_M), 0, -1)
    view[reachable == 0] = view[reachable == 0] // 3
    for radius in (MIN_REACH_M, MAX_REACH_M):
        cv2.circle(
            view, view_point((0, 0)), int(radius * PX_PER_M), (0, 200, 0), 2
        )

    for i in range(-6, 7):
        along = i * GRID_M
        cv2.line(view, view_point((along, -0.6)), view_point((along, 0.6)), (90, 90, 90), 1)
        cv2.line(view, view_point((-0.6, along)), view_point((0.6, along)), (90, 90, 90), 1)
    for i in range(0, 7):
        cv2.putText(
            view,
            f"x={i * GRID_M:.1f}",
            view_point((i * GRID_M, -0.58)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )

    inverse = np.linalg.inv(homography)
    for d in detections:
        corners = np.array([[c.x, c.y, 1.0] for c in d.corners]) @ inverse.T
        corners = corners[:, :2] / corners[:, 2:]
        pts = np.array([view_point(c) for c in corners], np.int32)
        cv2.polylines(view, [pts], True, (0, 255, 0), 2)
        cv2.putText(
            view,
            f"tag {d.id}",
            tuple(pts[0]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

    cv2.drawMarker(view, view_point((0, 0)), (255, 255, 255), cv2.MARKER_DIAMOND, 24, 2)
    if tool is not None:
        cv2.drawMarker(view, view_point(tool), (255, 128, 0), cv2.MARKER_TILTED_CROSS, 30, 3)
    if target is not None:
        cv2.drawMarker(view, view_point(target), (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
    cv2.putText(view, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.click_move")
    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")

    arm = ArmControl(ur)
    scene = SceneManager(ur)
    camera = CameraView()

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(scene)
    executor.add_node(camera)
    threading.Thread(target=executor.spin, daemon=True).start()

    params = PlanRequestParameters(ur, "ompl")
    params.max_velocity_scaling_factor = 0.3
    params.max_acceleration_scaling_factor = 0.3

    targets = queue.Queue(maxsize=1)
    state = {"status": "click the table to move there", "target": None}
    done = threading.Event()

    def worker():
        while not done.is_set():
            try:
                x, y = targets.get(timeout=0.2)
            except queue.Empty:
                continue
            state["status"] = f"moving to x={x:.3f} y={y:.3f}"
            if arm.go_to_pose(x, y, HOVER_HEIGHT_M, *EE_DOWN, params):
                state["status"] = f"at x={x:.3f} y={y:.3f}"
            else:
                state["status"] = f"could not reach x={x:.3f} y={y:.3f}"
            logger.info(state["status"])

    def on_mouse(event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if not targets.empty() or state["status"].startswith("moving"):
            return
        hit = view_to_world(u, v)
        reach = math.hypot(*hit)
        if not MIN_REACH_M <= reach <= MAX_REACH_M:
            state["status"] = f"x={hit[0]:.2f} y={hit[1]:.2f} is out of reach"
            return
        state["target"] = hit
        targets.put(hit)

    threading.Thread(target=worker, daemon=True).start()

    try:
        scene.add_box(
            "table",
            "base_link",
            [2.0, 2.0, 0.01],
            (0.0, 0.0, -0.01),
        )

        cv2.namedWindow(WINDOW)
        cv2.setMouseCallback(WINDOW, on_mouse)
        while rclpy.ok() and not arm.stop_event.is_set():
            snap = camera.snapshot()
            if snap is not None:
                image, stamp, frame_id = snap
                homography = camera.table_homography(stamp, frame_id)
                if homography is not None:
                    view = topdown(image, homography)
                    draw(
                        view,
                        homography,
                        camera.detections,
                        state["target"],
                        camera.tool_xy(),
                        state["status"],
                    )
                    cv2.imshow(WINDOW, view)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    finally:
        done.set()
        cv2.destroyAllWindows()
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
