"""
Scores wrist camera mounts against the touched circles.

For each mount, the base -> tool0 pose and the mount give the camera's pose in
base_link. Each detected circle centre becomes a ray, which is intersected with
the table plane at the circle's touched height and compared with where the
nozzle touched it.

    score_wrist_mounts_against_touched_circles <view dir> [touched_points.yaml]

The views are from capture_wrist_views_of_touched_circles
(camera_pose_calibration/data/wrist_mount_check_with_touched_circle_views).
"""

import glob
import os
import sys

import numpy as np

from colman_perception import plan_wrist_views_of_touched_circles as plan


def observations(view_dir):
    obs = []
    for path in sorted(glob.glob(f"{view_dir}/view_*.npz")):
        d = np.load(path)
        for label, px in zip(d["labels"], d["pixels"]):
            obs.append(
                dict(
                    view=os.path.basename(path),
                    label=int(label),
                    uv=px[:2],
                    resid=px[2],
                    tool0=d["tool0"],
                    k=d["k"],
                )
            )
    return obs


def to_table(o, mount, z):
    """Where the ray through the circle centre meets the plane at z, in base_link."""
    cam = o["tool0"] @ mount
    ray = np.linalg.inv(o["k"]) @ np.array([o["uv"][0], o["uv"][1], 1.0])
    ray_base = cam[:3, :3] @ ray
    t = (z - cam[2, 3]) / ray_base[2]
    return (cam[:3, 3] + t * ray_base)[:2]


def errors(obs, mount, pts, dz=0.0):
    """Projected minus touched, in mm, for each observation."""
    out = []
    for o in obs:
        truth = pts[o["label"]]
        out.append((to_table(o, mount, truth[2] + dz) - truth[:2]) * 1000)
    return np.array(out)


def summary(e):
    norm = np.linalg.norm(e, axis=1)
    return (
        f"mean x {e[:, 0].mean():+5.2f} y {e[:, 1].mean():+5.2f} mm   "
        f"rms {np.sqrt((e**2).sum(1).mean()):5.2f}   "
        f"median {np.median(norm):5.2f}   worst {norm.max():5.2f} mm"
    )


def per_circle_rms(obs, e):
    labels = sorted({o["label"] for o in obs})
    rms = {}
    for label in labels:
        rows = [i for i, o in enumerate(obs) if o["label"] == label]
        rms[label] = round(float(np.sqrt((e[rows] ** 2).sum(1).mean())), 2)
    return rms


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    pts = plan.circles(sys.argv[2] if len(sys.argv) > 2 else plan.TOUCHED)
    obs = observations(sys.argv[1])
    counts = {}
    for o in obs:
        counts[o["label"]] = counts.get(o["label"], 0) + 1
    views = len({o["view"] for o in obs})
    per_circle = dict(sorted(counts.items()))
    print(f"{len(obs)} observations in {views} views; per circle {per_circle}")
    for dz_mm in (0.0, 2.0):
        print(f"\nplane at the touched height + {dz_mm:.0f} mm (nozzle compression)")
        for name, m in plan.MOUNTS.items():
            print(f"  {name:50s} {summary(errors(obs, m, pts, dz_mm / 1000))}")
    print("\nper circle (no allowance), rms mm:")
    for name, m in plan.MOUNTS.items():
        print(f"  {name:50s} {per_circle_rms(obs, errors(obs, m, pts))}")


if __name__ == "__main__":
    main()
