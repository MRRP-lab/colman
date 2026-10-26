import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from colman_motion.arm_control import ArmControl
from colman_motion.click_move import level_camera_rotation, quaternion_to_matrix
from colman_motion.demo_camera import rotation_to_quaternion
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager

BASE_FRAME = "base_link"
TOOL_FRAME = "tool0"
CAMERA_FRAME = "oakd_pro_camera_rgb_optical_frame_wrist"

# The tabletop in the base frame: x from NEAR to FAR, y from -HALF to +HALF.
TABLE_NEAR_X_M = 0.09
TABLE_FAR_X_M = 0.63
TABLE_HALF_WIDTH_M = 0.5625

# The camera sits at most this far from the base, which is about as far as
# the arm reaches with the camera held out in front of the hand.
MAX_CAMERA_RADIUS_M = 0.38
STANDOFF_M = 0.2

CAMERA_HEIGHTS_M = (0.22, 0.32)

# Points to look at, relative to a corner: toward the middle of the table, and
# along each of the two edges that meet there.
LOOK_INWARD_M = 0.15
LOOK_ALONG_M = 0.25

# How long the arm sits still before data is taken, and how many depth frames
# are collected for the median.
SETTLE_S = 1.5
DEPTH_FRAMES = 10

# The arm must not creep more than this while the data is taken.
MAX_DRIFT_M = 0.0005


def matrix(position, rotation):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = position
    return m


class Recorder(Node):
    """
    Keeps the latest color image, the recent depth images and camera info, and
    looks up frames.
    """

    def __init__(self):
        super().__init__("calib_recorder")
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.rgb = None
        self.rgb_k = None
        self.depth_k = None
        self.depths = []
        self.collecting = False
        self.create_subscription(
            Image, "/oakd_pro/rgb/image", self.rgb_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/oakd_pro/depth/image", self.depth_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/oakd_pro/rgb/camera_info",
            self.rgb_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/oakd_pro/depth/camera_info",
            self.depth_info_cb,
            qos_profile_sensor_data,
        )

    def rgb_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.rgb = image

    def depth_cb(self, msg):
        if not self.collecting:
            return
        depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        with self.lock:
            self.depths.append(depth)

    def rgb_info_cb(self, msg):
        self.rgb_k = np.array(msg.k).reshape(3, 3)

    def depth_info_cb(self, msg):
        self.depth_k = np.array(msg.k).reshape(3, 3)

    def pose(self, parent, child):
        """
        The 4x4 transform of child in parent, or None.
        """
        try:
            transform = self.buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        t = transform.transform.translation
        r = transform.transform.rotation
        return matrix([t.x, t.y, t.z], quaternion_to_matrix((r.x, r.y, r.z, r.w)))

    def capture(self):
        """
        Collects one still view, or returns None if the arm moved or data is missing.
        """
        before = self.pose(BASE_FRAME, TOOL_FRAME)
        with self.lock:
            self.depths = []
        self.collecting = True
        deadline = time.time() + 10.0
        while time.time() < deadline:
            with self.lock:
                if len(self.depths) >= DEPTH_FRAMES:
                    break
            time.sleep(0.05)
        self.collecting = False
        after = self.pose(BASE_FRAME, TOOL_FRAME)
        with self.lock:
            depths = list(self.depths)
            rgb = None if self.rgb is None else self.rgb.copy()
        camera_in_tool = self.pose(TOOL_FRAME, CAMERA_FRAME)
        missing = [
            name
            for name, value in (
                ("tool pose", before),
                ("tool pose after", after),
                ("camera in tool", camera_in_tool),
                ("color image", rgb),
                ("color info", self.rgb_k),
                ("depth info", self.depth_k),
            )
            if value is None
        ]
        if len(depths) < DEPTH_FRAMES:
            missing.append(f"depth frames ({len(depths)}/{DEPTH_FRAMES})")
        if missing:
            self.get_logger().warn("Missing " + ", ".join(missing))
            return None
        drift = np.linalg.norm(after[:3, 3] - before[:3, 3])
        if drift > MAX_DRIFT_M:
            self.get_logger().warn(f"The arm moved {drift * 1000:.2f} mm")
            return None
        stack = np.array(depths, dtype=np.float32)
        stack[stack == 0] = np.nan
        with np.errstate(all="ignore"):
            depth = np.nan_to_num(np.nanmedian(stack, axis=0)).astype(np.uint16)
        return {
            "rgb": rgb,
            "depth": depth,
            "base_from_tool": after,
            "tool_from_camera": camera_in_tool,
            "rgb_k": self.rgb_k,
            "depth_k": self.depth_k,
        }


def views():
    """
    The camera positions and the points on the table they look at, as
    (camera, target) pairs in the base frame.
    """
    corners = [
        (x, y)
        for x in (TABLE_NEAR_X_M, TABLE_FAR_X_M)
        for y in (-TABLE_HALF_WIDTH_M, TABLE_HALF_WIDTH_M)
    ]
    middle = np.array([(TABLE_NEAR_X_M + TABLE_FAR_X_M) / 2.0, 0.0])
    for corner in map(np.array, corners):
        outward = corner / np.linalg.norm(corner)
        radius = min(np.linalg.norm(corner) - STANDOFF_M, MAX_CAMERA_RADIUS_M)
        inward = (middle - corner) / np.linalg.norm(middle - corner)
        along_x = np.array([-np.sign(corner[0] - middle[0]), 0.0])
        along_y = np.array([0.0, -np.sign(corner[1])])
        for height in CAMERA_HEIGHTS_M:
            camera = np.array([*(outward * radius), height])
            for toward, distance in (
                (inward, LOOK_INWARD_M),
                (along_x, LOOK_ALONG_M),
                (along_y, LOOK_ALONG_M),
            ):
                yield camera, np.array([*(corner + distance * toward), 0.0])


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.calib_sweep")
    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")

    arm = ArmControl(ur)
    scene = SceneManager(ur)
    recorder = Recorder()

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(scene)
    executor.add_node(recorder)
    threading.Thread(target=executor.spin, daemon=True).start()

    params = PlanRequestParameters(ur, "ompl")
    params.max_velocity_scaling_factor = 0.3
    params.max_acceleration_scaling_factor = 0.3

    output = os.environ.get("CALIB_DIR", "calib_data")
    os.makedirs(output, exist_ok=True)

    try:
        scene.add_box("table", "base_link", [2.0, 2.0, 0.01], (0.0, 0.0, -0.01))

        start = recorder.pose(TOOL_FRAME, CAMERA_FRAME)
        while start is None or recorder.pose(BASE_FRAME, TOOL_FRAME) is None:
            time.sleep(0.2)
            start = recorder.pose(TOOL_FRAME, CAMERA_FRAME)
        # Where the camera sits on the hand, which never changes.
        camera_in_tool = start
        camera_rotation_in_tool = camera_in_tool[:3, :3]
        camera_origin_in_tool = camera_in_tool[:3, 3]

        saved = 0
        for index, (camera, target) in enumerate(views()):
            direction = target - camera
            direction /= np.linalg.norm(direction)
            camera_rotation = level_camera_rotation(direction)
            tool_rotation = camera_rotation @ camera_rotation_in_tool.T
            tool_position = camera - tool_rotation @ camera_origin_in_tool
            quaternion = rotation_to_quaternion(*tool_rotation.T)
            if not arm.go_to_pose(*tool_position, *quaternion, params):
                logger.warn(f"View {index}: no plan")
                continue
            time.sleep(SETTLE_S)
            data = recorder.capture()
            if data is None:
                logger.warn(
                    f"View {index}: no data, the arm moved or a stream is missing"
                )
                continue
            cv2.imwrite(f"{output}/view_{index:02d}_rgb.png", data.pop("rgb"))
            np.savez(f"{output}/view_{index:02d}.npz", **data)
            saved += 1
            logger.info(f"View {index}: saved ({saved} so far)")
        logger.info(f"Done, {saved} views in {output}")
    finally:
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
