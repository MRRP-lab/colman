"""Moves the arm through the planned views and records, for each view, where the wrist
camera sees the touched circles and the base -> tool0 pose.

    wrist_circle_sweep <out dir>      (run on the domain of the arm stack)

The touched circle positions are read from calibration/data/overhead_touch_off.

Needs the driver, the motion server and a wrist camera_publisher running.
"""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from colman_interfaces.action import MovePose
from colman_perception import wrist_circle_plan as plan

SETTLE_S = 2.5
FRAMES = 3
MAX_TRIES = 45
RING_R_MM = 11.3  # drawn circle radius, from the overhead image


def quat_xyzw(m):
    w = np.sqrt(max(1.0 + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2
    return [
        (m[2, 1] - m[1, 2]) / (4 * w),
        (m[0, 2] - m[2, 0]) / (4 * w),
        (m[1, 0] - m[0, 1]) / (4 * w),
        w,
    ]


def detect_ring(gray, predicted, dist_m, k):
    """Centre of the drawn ring near a predicted pixel, or None.

    Hough finds the ring, then an ellipse is fitted to its dark pixels for the centre.
    Returns (u, v, fit residual in px, ring radius in px).
    """
    f = k[0, 0]
    r_px = RING_R_MM / 1000 * f / dist_m
    x0, y0 = int(predicted[0]), int(predicted[1])
    half = int(r_px * 2.4)
    top, left = max(y0 - half, 0), max(x0 - half, 0)
    win = gray[top : y0 + half, left : x0 + half]
    if win.shape[0] < 50 or win.shape[1] < 50:
        return None
    blur = cv2.GaussianBlur(win, (0, 0), 2.5)
    c = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=r_px,
        param1=100,
        param2=25,
        minRadius=int(r_px * 0.6),
        maxRadius=int(r_px * 1.5),
    )
    if c is None:
        return None
    cx, cy, r = sorted(
        c[0], key=lambda q: (q[0] - (x0 - left)) ** 2 + (q[1] - (y0 - top)) ** 2
    )[0]
    ys, xs = np.mgrid[0 : win.shape[0], 0 : win.shape[1]]
    d = np.hypot(xs - cx, ys - cy)
    ring = np.abs(d - r) < max(6, 0.12 * r)
    w = win.astype(float)
    bg = np.median(w[ring])
    dark = ring & (w < bg - 0.45 * (bg - w[ring].min()))
    pts = np.c_[xs[dark], ys[dark]].astype(np.float32)
    if len(pts) < 30:
        return None
    (ex, ey), (a, b), ang = cv2.fitEllipse(pts)
    resid = np.sqrt(
        np.mean(((np.hypot(pts[:, 0] - ex, pts[:, 1] - ey)) - 0.5 * (a + b) / 2) ** 2)
    )
    return float(ex + left), float(ey + top), float(resid), float(0.5 * (a + b) / 2)


class Sweep(Node):
    def __init__(self):
        super().__init__("wrist_circle_sweep")
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.image = None
        self.info = None
        self.create_subscription(
            Image,
            "/oakd_pro/rgb/image",
            lambda m: setattr(self, "image", (time.time(), m)),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/oakd_pro/rgb/camera_info",
            lambda m: setattr(self, "info", m),
            qos_profile_sensor_data,
        )
        self.client = ActionClient(self, MovePose, "move_pose")

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move(self, tip):
        goal = MovePose.Goal()
        goal.slow = True
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in tip[:3, 3])
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat_xyzw(
            tip
        )
        goal.pose = p
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            return False, "rejected"
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        r = res.result().result
        return r.success, r.message

    def tool_pose(self):
        t = self.buffer.lookup_transform("base_link", "tool0", Time()).transform
        x, y, z, w = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
        m = np.eye(4)
        m[:3, :3] = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return m

    def frames(self):
        self.spin_for(SETTLE_S)
        start, out = time.time(), []
        while time.time() - start < 8.0 and len(out) < FRAMES:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image and self.image[0] > start:
                out.append(self.bridge.imgmsg_to_cv2(self.image[1], "bgr8"))
                self.image = None
        return out


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    pts = plan.circles()
    cands = plan.candidates(pts)
    count = {label: 0 for label in pts}
    saved = 0
    for name in sorted(os.listdir(out)):  # carry on from views already saved here
        if name.endswith(".npz"):
            saved += 1
            for label in np.load(f"{out}/{name}")["labels"]:
                count[int(label)] += 1
    print(f"{saved} views already saved, coverage {count}")
    rclpy.init()
    node = Sweep()
    if not node.client.wait_for_server(timeout_sec=15.0):
        sys.exit("move_pose server not available")
    while node.info is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    k = np.array(node.info.k).reshape(3, 3)
    tries = 0
    while tries < MAX_TRIES:
        index = plan.next_view(cands, count)
        if index is None:
            break
        v = cands.pop(index)
        tries += 1
        ok, msg = node.move(v["tip"])
        if not ok:
            print(f"try {tries}: move failed ({msg}) for {v['params']}")
            gx, gy, dist, tilt, az, _ = v["params"]
            # a pose that cannot be reached usually means its neighbours cannot either
            cands = [c for c in cands if c["params"][:5] != (gx, gy, dist, tilt, az)]
            continue
        frames = node.frames()
        if not frames:
            print(f"try {tries}: no image")
            continue
        pose = node.tool_pose()
        cam = pose @ plan.AIM_MOUNT
        found = {}
        preview = frames[-1].copy()
        for label, uv in plan.visible(cam, pts).items():
            inv = np.linalg.inv(cam)
            dist = np.linalg.norm(inv[:3, :3] @ pts[label] + inv[:3, 3])
            hits = [
                detect_ring(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), uv, dist, k)
                for f in frames
            ]
            hits = [h for h in hits if h]
            if not hits:
                print(f"  circle {label}: ring not found near {np.round(uv)}")
                continue
            found[label] = np.array(hits).mean(0)
            cv2.drawMarker(
                preview,
                (int(found[label][0]), int(found[label][1])),
                (0, 0, 255),
                cv2.MARKER_CROSS,
                60,
                4,
            )
            cv2.putText(
                preview,
                str(label),
                (int(found[label][0]) + 40, int(found[label][1]) - 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                3,
                (0, 255, 255),
                6,
            )
        np.savez(
            f"{out}/view_{saved:02d}.npz",
            tool0=pose,
            k=k,
            labels=list(found),
            pixels=np.array(list(found.values())) if found else np.zeros((0, 4)),
            params=v["params"],
        )
        cv2.imwrite(f"{out}/view_{saved:02d}.jpg", cv2.resize(preview, (960, 540)))
        for label in found:
            count[label] += 1
        print(
            f"view saved as {saved}: circles {sorted(found)} params {v['params']}"
            f"  coverage {count}"
        )
        saved += 1
    print(f"{saved} views saved in {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
