"""
Fits the overhead camera's pose to points the arm has touched.

The touched points are ground truth in base_link (from the arm's forward
kinematics). Their pixels come from click_touch_off_marks or the circle finder, and
the camera's intrinsics from the capture. The pose is the only thing fitted:
translation and a rotation about the starting pose. Points are projected onto
one plane at their mean touched height.

    fit_overhead_pose_to_touched_points <capture stem> <touched_points.yaml> \
        <mark_pixels.yaml> <start_overhead_pose.yaml> <out.yaml>

The stem has <stem>.npz (rgb_k, the intrinsics) from
capture_undistorted_camera_still. The start pose is an
overhead_camera_pose_in_base_link.yaml. Prints the error of the start pose and of the
fitted pose, then a leave-one-out check (the error of each point when it is the only one
left out of the fit), and writes the fitted pose in the same format.
"""

import sys

import cv2
import numpy as np
import yaml

from colman_perception.check_overhead_pose_against_touched_points import table_points
from colman_perception.fit_wrist_mount_to_table_edges import least_squares


def quaternion_xyzw(m):
    w = np.sqrt(max(1 + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2
    return [
        (m[2, 1] - m[1, 2]) / (4 * w),
        (m[0, 2] - m[2, 0]) / (4 * w),
        (m[1, 0] - m[0, 1]) / (4 * w),
        w,
    ]


def main():
    if len(sys.argv) != 6:
        sys.exit(__doc__)
    stem, touched_path, pixels_path, start_path, out_path = sys.argv[1:]

    start = yaml.safe_load(open(start_path))
    rotation0 = np.array(start["rotation_matrix"])
    translation0 = np.array(start["translation_m"])
    k = np.load(f"{stem}.npz")["rgb_k"]
    marks = {
        str(m["label"]): m["pixel"] for m in yaml.safe_load(open(pixels_path))["points"]
    }
    points = yaml.safe_load(open(touched_path))["points"]
    labels = [p["label"] for p in points]
    xyz = np.array([p["xyz"] for p in points])
    pixels = np.array([marks[str(label)] for label in labels])
    plane = xyz[:, 2].mean()
    world = np.c_[xyz[:, :2], np.full(len(xyz), plane)]

    def pose(x):
        return cv2.Rodrigues(x[:3])[0] @ rotation0, translation0 + x[3:]

    def residuals(x, index=None):
        rotation, translation = pose(x)
        sel = slice(None) if index is None else index
        camera = (rotation.T @ (world[sel] - translation).T).T
        uv = (k @ camera.T).T
        return (uv[:, :2] / uv[:, 2:3] - pixels[sel]).ravel()

    def errors_mm(x, index=None):
        rotation, translation = pose(x)
        sel = slice(None) if index is None else index
        return (
            table_points(pixels[sel], k, rotation, translation, plane) - world[sel, :2]
        ) * 1000

    def report(name, x):
        e = errors_mm(x)
        total = np.sqrt((e**2).sum(1).mean())
        print(
            f"{name}: mean x error {e[:, 0].mean():+.2f} mm, rms {total:.2f} mm total, "
            f"worst {np.linalg.norm(e, axis=1).max():.2f} mm"
        )

    x0 = np.zeros(6)
    report("start pose ", x0)
    x = least_squares(residuals, x0, f_scale=1.0, iterations=200)
    report("fitted pose", x)

    held = []
    for i in range(len(labels)):
        rest = [j for j in range(len(labels)) if j != i]
        xi = least_squares(lambda v: residuals(v, rest), x, f_scale=1.0, iterations=100)
        held.append(np.linalg.norm(errors_mm(xi, [i])))
    print(
        f"leave-one-out error of the held-out point: mean {np.mean(held):.2f} mm, "
        f"worst {np.max(held):.2f} mm"
    )

    rotation, translation = pose(x)
    print(
        "change: translation "
        f"{np.round((translation - translation0) * 1000, 1).tolist()} mm, "
        f"rotation {np.degrees(np.linalg.norm(x[:3])):.3f} deg, "
        f"plane z {plane * 1000:.1f} mm"
    )
    q = quaternion_xyzw(rotation)
    out = dict(
        parent_frame="base_link",
        child_frame="overhead_camera_optical_frame",
        translation_m=[float(v) for v in translation],
        quaternion_xyzw=[float(v) for v in q],
        rotation_matrix=[[float(v) for v in row] for row in rotation],
        static_transform_args=[f"{v:.5f}" for v in list(translation) + q]
        + ["base_link", "overhead_camera_optical_frame"],
        fit=dict(
            method="touched points, camera pose only (intrinsics fixed), projected "
            "onto the mean touched height",
            points=len(labels),
            plane_z_m=float(plane),
            note="touched z includes the nozzle's rubber compression",
        ),
    )
    yaml.safe_dump(out, open(out_path, "w"), sort_keys=False)


if __name__ == "__main__":
    main()
