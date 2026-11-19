"""
Plans wrist camera views that each see several of the touched circles.

Offline: uses the touched circle positions and the previous wrist mount (the hand-eye
fit c8a4f18) to predict which circles land in the image, then greedily picks views
until every circle is seen from several arm poses. Shared by
capture_wrist_views_of_touched_circles and score_wrist_mounts_against_touched_circles.
"""

import itertools

import numpy as np
import yaml

W, H = 3840, 2160
K0 = np.array([[2915.0, 0, 1925.0], [0, 2914.0, 1094.0], [0, 0, 1.0]])  # 4K, approx
MARGIN_PX = 220  # a circle this near the image edge is not used
TIP_IN_TOOL = (0.0, 0.0455, 0.088)


def rpy(r, p, y):
    cx, sx, cy, sy, cz, sz = (
        np.cos(r),
        np.sin(r),
        np.cos(p),
        np.sin(p),
        np.cos(y),
        np.sin(y),
    )
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def mount(x, y, z, r, p, yaw):
    """tool0 -> camera optical frame, from the URDF oakd_pro_joint values."""
    m = np.eye(4)
    m[:3, :3] = rpy(r, p, yaw) @ rpy(0, 0, np.pi)  # the optical joint turns by pi
    m[:3, 3] = [x, y, z]
    return m


MOUNTS = {
    "original mount (before any fit)": mount(0.0, -0.100934, 0.040, 0, 0, np.pi),
    "hand-eye fit c8a4f18 (previous main)": mount(
        0.000333, -0.100725, 0.044019, 0.012657, -0.007433, -3.138355
    ),
    "grid fit (official wrist mount)": mount(
        0.000609, -0.099473, 0.043895, 0.009631, -0.003062, -3.141120
    ),
}
AIM_MOUNT = MOUNTS["hand-eye fit c8a4f18 (previous main)"]


TOUCHED = (
    "camera_pose_calibration/data/overhead_pose_from_touched_points/"
    "touched_points_set2_after_table_leg_fix.yaml"
)


def circles(path=TOUCHED):
    pts = yaml.safe_load(open(path))["points"]
    return {p["label"]: np.array(p["xyz"]) for p in pts}


def camera_pose(target, distance, tilt_deg, azimuth_deg, roll_deg):
    t, a = np.radians(tilt_deg), np.radians(azimuth_deg)
    offset = distance * np.array(
        [np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), np.cos(t)]
    )
    z = -offset / np.linalg.norm(offset)
    ref = np.array([-1.0, 0.0, 0.0])
    y = ref - z * (ref @ z)
    if np.linalg.norm(y) < 1e-6:
        y = np.array([0.0, 1.0, 0.0])
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    c, s = np.cos(np.radians(roll_deg)), np.sin(np.radians(roll_deg))
    roll = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    m = np.eye(4)
    m[:3, :3] = np.c_[x, y, z] @ roll
    m[:3, 3] = target + offset
    return m


def tip_pose_for(cam_in_base, mnt=AIM_MOUNT):
    tool = cam_in_base @ np.linalg.inv(mnt)
    tip = np.eye(4)
    tip[:3, 3] = TIP_IN_TOOL
    return tool @ tip


def project(cam_in_base, points, k=K0):
    inv = np.linalg.inv(cam_in_base)
    p = (inv[:3, :3] @ points.T).T + inv[:3, 3]
    uv = (k @ p.T).T
    return uv[:, :2] / uv[:, 2:3], p[:, 2]


def visible(cam_in_base, pts):
    labels = list(pts)
    uv, depth = project(cam_in_base, np.array([pts[label] for label in labels]))
    return {
        label: uv[i]
        for i, label in enumerate(labels)
        if depth[i] > 0.1
        and MARGIN_PX < uv[i, 0] < W - MARGIN_PX
        and MARGIN_PX < uv[i, 1] < H - MARGIN_PX
    }


def candidates(pts):
    out = []
    for gx, gy, dist, tilt, roll in itertools.product(
        (0.20, 0.30, 0.40),
        np.arange(-0.45, 0.46, 0.10),
        (0.30, 0.38),
        (0, 12, 22),
        (0, 90, -90, 180),
    ):
        for az in (0,) if tilt == 0 else (0, 90, 180, 270):
            cam = camera_pose(np.array([gx, gy, 0.0]), dist, tilt, az, roll)
            tip = tip_pose_for(cam)
            if tip[2, 3] < 0.09 or np.hypot(*cam[:2, 3]) > 0.40:
                continue
            seen = visible(cam, pts)
            if len(seen) >= 2:
                out.append(
                    dict(
                        cam=cam,
                        tip=tip,
                        seen=seen,
                        params=(gx, gy, dist, tilt, az, roll),
                    )
                )
    return out


def next_view(cands, count, per_circle=3):
    """The candidate that helps the least-seen circles most, or None if none helps."""

    def gain(c):
        return sum(
            max(per_circle - count[label], 0) for label in c["seen"]
        ) + 0.01 * len(c["seen"])

    if not cands:
        return None
    index = max(range(len(cands)), key=lambda i: gain(cands[i]))
    return index if gain(cands[index]) >= 0.5 else None


def plan(pts, per_circle=3, max_views=16):
    cands = candidates(pts)
    count = {label: 0 for label in pts}
    chosen = []
    while len(chosen) < max_views and min(count.values()) < per_circle:

        def gain(c):
            return sum(
                max(per_circle - count[label], 0) for label in c["seen"]
            ) + 0.01 * len(c["seen"])

        index = max(range(len(cands)), key=lambda i: gain(cands[i]))
        best = cands[index]
        if gain(best) < 0.5:
            break
        chosen.append(best)
        for label in best["seen"]:
            count[label] += 1
        cands.pop(index)
    return chosen, count


if __name__ == "__main__":
    pts = circles()
    chosen, count = plan(pts)
    print(len(chosen), "views; times each circle is seen:", count)
    for i, c in enumerate(chosen):
        gx, gy, d, t, a, r = c["params"]
        print(
            f"{i:2d} aim ({gx:.2f},{gy:+.2f}) dist {d} tilt {t:2d} az {a:3d} "
            f"roll {r:4d}  tip z {c['tip'][2, 3]:.3f}  sees {sorted(c['seen'])}"
        )
