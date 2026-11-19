"""
Moves the arm through slow views of a flat grid on the table and records, for
each view, the grid corners in the wrist image and the base -> tool0 pose.

Run on the domain of the running stack (driver, motion server, wrist
camera_publisher). The grid's position only needs to be roughly right, because
grid_fit_wrist.py fits it too.

Usage:
    python grid_sweep_wrist.py <out dir> <grid x> <grid y> [--dry-run]
"""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from colman_perception.grid_check_overhead import find_corners
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from colman_interfaces.action import MovePose

BASE, TOOL = "base_link", "tool0"
COLS, ROWS, SQUARE, BOARD_M = 7, 5, 0.031, 0.0047

# The vacuum tip and the camera, in tool0 (from the URDF). Used only to aim.
TIP_IN_TOOL = (0.0, 0.0455, 0.088)
CAMERA_IN_TOOL = (0.0003, -0.1007, 0.044)

# Views: distance from the grid, tilt from straight down, azimuth of the tilt,
# and roll about the optical axis.
DISTANCES_M = (0.26, 0.34)
TILTS_DEG = (0.0, 14.0, 24.0)
AZIMUTHS_DEG = (0, 90, 180, 270)
ROLLS_DEG = (0.0, 35.0, -35.0)

MIN_TIP_Z_M = 0.09  # never plan a view with the vacuum tip lower than this
MAX_CAMERA_RADIUS_M = 0.40
SETTLE_S = 2.5
MIN_CORNER_FRAMES = 3


def matrix(rotation, position):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = position
    return m


def rodrigues(axis, degrees):
    return cv2.Rodrigues(np.asarray(axis, float) * np.radians(degrees))[0]


def camera_pose(target, distance, tilt, azimuth, roll):
    """Camera looking at target from distance, tilted from vertical; camera -> base."""
    t, a = np.radians(tilt), np.radians(azimuth)
    offset = distance * np.array(
        [np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), np.cos(t)]
    )
    z = -offset / np.linalg.norm(offset)
    # Camera y points toward the base at zero roll, which puts the image upright.
    ref = np.array([-1.0, 0.0, 0.0])
    y = ref - z * (ref @ z)
    if np.linalg.norm(y) < 1e-6:
        y = np.array([0.0, 1.0, 0.0])
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    rotation = np.c_[x, y, z] @ rodrigues([0, 0, 1], roll)
    return matrix(rotation, target + offset)


def tip_pose_for(camera_in_base):
    """The pose of the vacuum tip that puts the camera at camera_in_base."""
    tool_in_camera = np.linalg.inv(matrix(np.eye(3), CAMERA_IN_TOOL))
    tool = camera_in_base @ tool_in_camera
    return tool @ matrix(np.eye(3), TIP_IN_TOOL)


def quaternion_xyzw(m):
    w = np.sqrt(max(1.0 + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2
    return [
        (m[2, 1] - m[1, 2]) / (4 * w),
        (m[0, 2] - m[2, 0]) / (4 * w),
        (m[1, 0] - m[0, 1]) / (4 * w),
        w,
    ]


def plan_views(grid_xy):
    target = np.array([grid_xy[0], grid_xy[1], BOARD_M])
    views = []
    for distance in DISTANCES_M:
        for tilt in TILTS_DEG:
            for azimuth in AZIMUTHS_DEG if tilt else (0,):
                for roll in ROLLS_DEG:
                    cam = camera_pose(target, distance, tilt, azimuth, roll)
                    tip = tip_pose_for(cam)
                    if tip[2, 3] < MIN_TIP_Z_M:
                        continue
                    if np.hypot(*cam[:2, 3]) > MAX_CAMERA_RADIUS_M:
                        continue
                    views.append((distance, tilt, azimuth, roll, tip))
    return views


class Sweep(Node):
    def __init__(self):
        super().__init__("grid_sweep_wrist")
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.image = None
        self.info = None
        self.create_subscription(
            Image, "/oakd_pro/rgb/image", self.on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/oakd_pro/rgb/camera_info",
            self.on_info,
            qos_profile_sensor_data,
        )
        self.client = ActionClient(self, MovePose, "move_pose")

    def on_image(self, msg):
        self.image = (time.time(), msg)

    def on_info(self, msg):
        self.info = msg

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move(self, tip):
        goal = MovePose.Goal()
        goal.slow = True
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in tip[:3, 3])
        q = quaternion_xyzw(tip)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        goal.pose = p
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            return False, "rejected"
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        res = result.result().result
        return res.success, res.message

    def tool_pose(self):
        t = self.buffer.lookup_transform(BASE, TOOL, Time()).transform
        q, v = t.rotation, t.translation
        rot = cv2.Rodrigues  # placeholder to keep imports local
        x, y, z, w = q.x, q.y, q.z, q.w
        r = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        del rot
        return matrix(r, [v.x, v.y, v.z])

    def capture(self):
        """Corners from fresh frames after the arm has settled, or None."""
        self.spin_for(SETTLE_S)
        start = time.time()
        found = []
        while time.time() - start < 6.0 and len(found) < MIN_CORNER_FRAMES:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image is None or self.image[0] < start:
                continue
            stamp, msg = self.image
            self.image = None
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            corners = find_corners(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), COLS, ROWS)
            if corners is not None:
                found.append((corners, frame))
        if len(found) < MIN_CORNER_FRAMES:
            return None
        return np.mean([c for c, _ in found], axis=0), found[-1][1]


def main():
    out = sys.argv[1]
    grid_xy = (float(sys.argv[2]), float(sys.argv[3]))
    dry = "--dry-run" in sys.argv
    views = plan_views(grid_xy)
    print(f"{len(views)} views planned")
    if dry:
        for n, (d, t, a, r, tip) in enumerate(views):
            print(n, d, t, a, r, np.round(tip[:3, 3], 3))
        return
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    node = Sweep()
    if not node.client.wait_for_server(timeout_sec=10.0):
        sys.exit("move_pose server not available")
    while node.info is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    k = np.array(node.info.k).reshape(3, 3)
    saved = 0
    for n, (d, t, a, r, tip) in enumerate(views):
        ok, message = node.move(tip)
        if not ok:
            print(f"view {n} (d={d} tilt={t} az={a} roll={r}): move failed ({message})")
            continue
        got = node.capture()
        if got is None:
            print(f"view {n}: grid not found")
            continue
        corners, frame = got
        pose = node.tool_pose()
        np.savez(
            f"{out}/view_{saved:02d}.npz",
            corners=corners,
            k=k,
            base_tool=pose,
            plan=[d, t, a, r],
        )
        cv2.imwrite(f"{out}/view_{saved:02d}.jpg", cv2.resize(frame, (960, 540)))
        print(f"view {n} saved as {saved}: d={d} tilt={t} az={a} roll={r}")
        saved += 1
    print(f"{saved} views saved in {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
