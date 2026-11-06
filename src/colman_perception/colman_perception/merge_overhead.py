"""
Fits the overhead camera pose to the table edges and the grid corners together.

The table edges (known lines in base_link, from the wrist calibration) tie the
camera's absolute position and yaw. The grids, each at its own unknown spot on
the table, pin down the camera's tilt and height. One six-parameter pose is
fitted to both at once, so neither has to be patched after the fact.

Usage:
    python merge_overhead.py <wrist calibration.yaml> <out.yaml> \
        <edge capture stem> [...] -- <grid capture stem> [...]
Each stem has <stem>.npz and <stem>_rgb.png (from capture_grid.py). The edge
captures are clear views of the whole table; the grid captures have a grid on it.

Compares three poses: edges only (what calibrate_overhead.py gives), grids only
(starting from the edge pose, camera x, y and yaw held), and the joint fit. Each is
scored on the grid placements (scale and rms against a perfect flat grid, and
leave-one-out for the joint fit) and on the table edges.
"""

import sys

import cv2
import numpy as np
import yaml

from colman_perception.calibrate_camera import least_squares
from colman_perception.calibrate_overhead import (
    SEED_CORNERS,
    SEED_EDGES,
    corner_pose,
    edge_residuals,
    find_edge,
    pose_from_theta,
    project,
    rotation_to_quaternion,
    table_edges,
)
from colman_perception.grid_check_overhead import fit_grid, table_points
from colman_perception.grid_fit_overhead import (
    HEIGHT,
    MODEL,
    grid_points,
    load,
    start_pose,
)

# Expected scatter of each kind of measurement, in pixels. The edge lines fit
# to about 0.4 px; sub-pixel grid corners are good to about 0.15 px.
SIGMA_EDGE_PX = 0.4
SIGMA_CORNER_PX = 0.15


NEAR_EDGE = 3  # x = 0.09: hand-measured only, and about 24 mm off, so it is not fitted


def detect_edges(stem, skip_near=True):
    image = cv2.imread(f"{stem}_rgb.png")
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (0, 0), 1.0)
    gray = gray.astype(np.float32)
    rng = np.random.default_rng(0)
    detected = {}
    for index, (a, b) in SEED_EDGES.items():
        if skip_near and index == NEAR_EDGE:
            continue
        found = find_edge(gray, SEED_CORNERS[a], SEED_CORNERS[b], rng)
        if found is not None:
            detected[index] = found["points"]
    return np.load(f"{stem}.npz")["rgb_k"], detected


def fit(theta0, edge_views, grids, edges, edge_weight=1.0, grid_weight=1.0):
    """
    Fits the camera pose (and each grid's own pose). With no edge views or no
    grids that part is left out. Returns theta.
    """
    starts = []
    for k, pixels in grids:
        rotation, translation = pose_from_theta(theta0)
        starts.append(start_pose(pixels, rotation, translation, k))
    n = len(grids)

    def residuals(x):
        rotation, translation = pose_from_theta(x[:6])
        out = []
        for k, detected in edge_views:
            out.append(
                edge_residuals(x[:6], k, detected, edges) * edge_weight / SIGMA_EDGE_PX
            )
        for i, (k, pixels) in enumerate(grids):
            pose = x[6 + 3 * i : 9 + 3 * i]
            predicted = project(
                k, rotation, translation, grid_points(pose, starts[i][1])
            )
            out.append(((predicted - pixels) * grid_weight / SIGMA_CORNER_PX).ravel())
        return np.concatenate(out)

    x0 = np.r_[theta0, np.concatenate([s[0] for s in starts])] if n else theta0
    return least_squares(lambda x, *_: residuals(x), x0, f_scale=1.0, iterations=300)[
        :6
    ]


def grid_scores(theta, grids):
    rotation, translation = pose_from_theta(theta)
    scores = []
    for k, pixels in grids:
        xy = table_points(pixels, k, rotation, translation, HEIGHT)
        scale, rms, _ = fit_grid(xy, MODEL)
        scores.append((scale, rms))
    return scores


def edge_scores(theta, edge_views, edges):
    """rms of the edge residuals, in pixels and in mm on the table."""
    r = np.concatenate([edge_residuals(theta, k, d, edges) for k, d in edge_views])
    rms_px = float(np.sqrt(np.mean(r**2)))
    return rms_px, rms_px * theta[5] / edge_views[0][0][0, 0] * 1000.0


def main():
    args = sys.argv[1:]
    wrist, out_path = args[:2]
    split = args.index("--")
    edge_stems, grid_stems = args[2:split], args[split + 1 :]

    edges = table_edges(wrist)
    edge_views = [detect_edges(s) for s in edge_stems]
    grids = [load(s) for s in grid_stems]

    k0, detected0 = edge_views[0]
    theta_edge = least_squares(
        lambda t, *_: edge_residuals(t, k0, detected0, edges),
        corner_pose(k0, edges),
        f_scale=1.0,
    )
    theta_edge = fit(theta_edge, edge_views, [], edges)
    theta_joint = fit(theta_edge, edge_views, grids, edges)

    poses = {"edges only": theta_edge, "joint (edges + grids)": theta_joint}
    print(f"{'pose':24s} {'camera x, y, z (m)':28s} edge rms px / mm")
    for name, theta in poses.items():
        _, translation = pose_from_theta(theta)
        px, mm = edge_scores(theta, edge_views, edges)
        print(f"{name:24s} {np.round(translation, 4)!s:28s} {px:.2f} / {mm:.2f}")
    r_edge, t_edge = pose_from_theta(theta_edge)
    r_joint, t_joint = pose_from_theta(theta_joint)
    tilt = np.degrees(np.arccos(np.clip((r_edge.T @ r_joint)[2, 2], -1, 1)))
    print(
        "joint minus edges-only: camera "
        f"{np.round((t_joint - t_edge) * 1000, 1)} mm, rotation {tilt:.2f} deg"
    )

    print("\ngrid placements: scale / rms mm")
    print(f"{'placement':20s} {'edges only':16s} {'joint':16s} {'joint, held out':16s}")
    edge_g, joint_g = grid_scores(theta_edge, grids), grid_scores(theta_joint, grids)
    held = []
    for i in range(len(grids)):
        theta = fit(theta_edge, edge_views, grids[:i] + grids[i + 1 :], edges)
        held.append(grid_scores(theta, [grids[i]])[0])
    for stem, a, b, c in zip(grid_stems, edge_g, joint_g, held):
        print(
            f"{stem.split('/')[-1]:20s} {a[0]:.4f} / {a[1]:.2f}   "
            f"{b[0]:.4f} / {b[1]:.2f}   {c[0]:.4f} / {c[1]:.2f}"
        )
    for name, scores in (
        ("edges only", edge_g),
        ("joint", joint_g),
        ("held out", held),
    ):
        s = np.array([x[0] for x in scores])
        print(f"scale spread, {name}: {100 * (s.max() - s.min()):.2f}%")

    print("\nweights: how much the joint pose moves if one kind of data counts more")
    for label, ew, gw in (
        ("edges x2", 2.0, 1.0),
        ("edges x0.5", 0.5, 1.0),
        ("grids x2", 1.0, 2.0),
    ):
        theta = fit(theta_edge, edge_views, grids, edges, ew, gw)
        _, translation = pose_from_theta(theta)
        rotation, _ = pose_from_theta(theta)
        deg = np.degrees(np.arccos(np.clip((r_joint.T @ rotation)[2, 2], -1, 1)))
        print(
            f"  {label:10s} camera {np.round((translation - t_joint) * 1000, 2)} mm, "
            f"rotation {deg:.3f} deg from the joint pose"
        )

    k_near, near = detect_edges(edge_stems[0], skip_near=False)
    if NEAR_EDGE in near:
        for name, (r, t) in (
            ("edges only", (r_edge, t_edge)),
            ("joint", (r_joint, t_joint)),
        ):
            xy = table_points(near[NEAR_EDGE], k_near, r, t, 0.0)
            print(
                f"near table edge from the image, {name}: "
                f"x = {xy[:, 0].mean():.4f} m (assumed {edges[3][0][0]:.4f})"
            )

    quaternion = rotation_to_quaternion(r_joint)
    out = {
        "parent_frame": "base_link",
        "child_frame": "overhead_camera_optical_frame",
        "translation_m": [float(v) for v in t_joint],
        "quaternion_xyzw": [float(v) for v in quaternion],
        "rotation_matrix": [[float(v) for v in row] for row in r_joint],
        "static_transform_args": [
            *(f"{v:.5f}" for v in t_joint),
            *(f"{v:.5f}" for v in quaternion),
            "base_link",
            "overhead_camera_optical_frame",
        ],
        "fit": {
            "method": "table edges and grid corners, one pose",
            "edges": edge_stems,
            "grids": grid_stems,
            "sigma_edge_px": SIGMA_EDGE_PX,
            "sigma_corner_px": SIGMA_CORNER_PX,
            "wrist_calibration": wrist,
        },
    }
    with open(out_path, "w") as f:
        yaml.safe_dump(out, f)
    print("\nwrote", out_path)


if __name__ == "__main__":
    main()
