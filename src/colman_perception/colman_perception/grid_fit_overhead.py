"""
Fits the overhead camera's tilt and height to flat grids lying at several
places on the table. A flat grid only gives the ratio of height to focal
length, so the focal length stays at its factory value.

Each placement has its own unknown pose (x, y, yaw) on the table, so the grids
cannot say where the camera is sideways or how it is turned about vertical.
Those stay at the values in the starting pose. What the grids do pin down is
how the apparent size of the squares changes across the table, which is the
camera's tilt and height. Tilting the camera also slides everything it sees
sideways, so the fitted pose is shifted back until the grids' average position
matches the starting pose.

Usage:
    python grid_fit_overhead.py <overhead_camera.yaml> <out.yaml> \
        <capture stem> [<capture stem> ...]
where each stem has <stem>.npz and <stem>_rgb.png (from capture_grid.py).

Prints, for each placement, the scale and rms of the corners against a perfect
flat grid (rigid fit, scale free) with the starting pose and with the fitted
pose, then a leave-one-out check: fit on all but one placement and score the
held-out one.
"""

import sys

import cv2
import numpy as np
import yaml

from colman_perception.calibrate_camera import least_squares
from colman_perception.grid_check_overhead import find_corners, fit_grid, table_points

COLS, ROWS = 7, 5
SQUARE = 0.031
HEIGHT = 0.004  # printed corners sit on a 4 mm board
MODEL = np.array([[i, j] for j in range(ROWS) for i in range(COLS)], float) * SQUARE


def load(stem):
    k = np.load(f"{stem}.npz")["rgb_k"]
    gray = cv2.cvtColor(cv2.imread(f"{stem}_rgb.png"), cv2.COLOR_BGR2GRAY)
    pixels = find_corners(gray, COLS, ROWS)
    if pixels is None:
        sys.exit(f"grid not found in {stem}")
    return k, pixels


def rotation_from_vector(vector):
    return cv2.Rodrigues(np.asarray(vector, dtype=np.float64))[0]


def quaternion_xyzw(m):
    w = np.sqrt(max(1.0 + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2
    return [
        (m[2, 1] - m[1, 2]) / (4 * w),
        (m[0, 2] - m[2, 0]) / (4 * w),
        (m[1, 0] - m[0, 1]) / (4 * w),
        w,
    ]


def camera(base_rotation, base_translation, params):
    """params = tilt about camera x, tilt about camera y, height change, focal scale."""
    rotation = base_rotation @ rotation_from_vector([params[0], params[1], 0])
    translation = base_translation + [0, 0, params[2]]
    return rotation, translation, params[3]


def project(rotation, translation, k, focal_scale, points):
    """base_link points (n, 3) to pixels."""
    cam = (points - translation) @ rotation  # rotation.T applied to row vectors
    pixels = cam[:, :2] / cam[:, 2:3]
    k = k.copy()
    k[0, 0] *= focal_scale
    k[1, 1] *= focal_scale
    return pixels * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]


def grid_points(pose, mirror):
    """Model corners placed on the table at (x, y, yaw), at the board height."""
    model = MODEL * [1, mirror]
    c, s = np.cos(pose[2]), np.sin(pose[2])
    xy = model @ np.array([[c, s], [-s, c]]) + pose[:2]
    return np.c_[xy, np.full(len(xy), HEIGHT)]


def start_pose(measured, rotation, translation, k):
    """A starting grid pose and mirror choice, from the starting camera pose."""
    xy = table_points(measured, k, rotation, translation, HEIGHT)
    best = None
    for mirror in (1, -1):
        for yaw in np.radians(np.arange(0, 360, 90)):
            pose = np.r_[
                xy.mean(0) - grid_points([0, 0, yaw], mirror)[:, :2].mean(0), yaw
            ]
            err = np.abs(grid_points(pose, mirror)[:, :2] - xy).max()
            if best is None or err < best[0]:
                best = (err, pose, mirror)
    return best[1], best[2]


def fit(
    captures, base_rotation, base_translation, free_camera=True, camera_params=None
):
    """
    Fits the camera parameters (if free_camera) and every placement's pose.
    Returns the camera parameters and the placement poses.
    """
    starts = [start_pose(p, base_rotation, base_translation, k) for k, p in captures]
    n = len(captures)
    x0 = np.r_[
        np.array([0, 0, 0, 1.0]) if camera_params is None else camera_params,
        np.concatenate([s[0] for s in starts]),
    ]
    n_cam = 3 if free_camera else 0

    def residuals(x, *_):
        params = np.r_[x[:3], 1.0] if free_camera else camera_params
        rotation, translation, scale = camera(base_rotation, base_translation, params)
        out = []
        for i, (k, pixels) in enumerate(captures):
            pose = x[n_cam + 3 * i : n_cam + 3 * i + 3]
            predicted = project(
                rotation, translation, k, scale, grid_points(pose, starts[i][1])
            )
            out.append((predicted - pixels).ravel())
        return np.concatenate(out)

    x0 = np.r_[x0[:3] if free_camera else [], x0[4:]]
    solution = least_squares(residuals, x0, f_scale=1.0, iterations=300)
    params = np.r_[solution[:3], 1.0] if free_camera else camera_params
    return params, solution[n_cam:].reshape(n, 3), [s[1] for s in starts]


def score(captures, base_rotation, base_translation, params):
    """(scale, rms mm) of each placement against a perfect grid, for a camera pose."""
    rotation, translation, focal_scale = camera(base_rotation, base_translation, params)
    out = []
    for k, pixels in captures:
        k = k.copy()
        k[0, 0] *= focal_scale
        k[1, 1] *= focal_scale
        xy = table_points(pixels, k, rotation, translation, HEIGHT)
        scale, rms, _ = fit_grid(xy, MODEL)
        out.append((scale, rms))
    return out


def main():
    start_path, out_path = sys.argv[1:3]
    stems = sys.argv[3:]
    pose = yaml.safe_load(open(start_path))
    base_rotation = np.array(pose["rotation_matrix"])
    base_translation = np.array(pose["translation_m"])
    captures = [load(stem) for stem in stems]

    params, _, _ = fit(captures, base_rotation, base_translation)
    zero = np.array([0, 0, 0, 1.0])
    before = score(captures, base_rotation, base_translation, zero)
    after = score(captures, base_rotation, base_translation, params)
    print("placement                scale before/after   rms mm before/after")
    for stem, b, a in zip(stems, before, after):
        print(
            f"{stem.split('/')[-1]:24s} {b[0]:.4f} / {a[0]:.4f}      "
            f"{b[1]:.2f} / {a[1]:.2f}"
        )
    print(
        f"fit: tilt x {np.degrees(params[0]):.3f} deg, "
        f"tilt y {np.degrees(params[1]):.3f} deg, "
        f"height {params[2] * 1000:+.1f} mm"
    )

    print("leave one out (fit without it, score it): scale / rms mm")
    for i, stem in enumerate(stems):
        rest = captures[:i] + captures[i + 1 :]
        p, _, _ = fit(rest, base_rotation, base_translation)
        s, r = score([captures[i]], base_rotation, base_translation, p)[0]
        s0, r0 = before[i]
        print(f"{stem.split('/')[-1]:24s} {s0:.4f} / {r0:.2f}  ->  {s:.4f} / {r:.2f}")

    rotation, translation, focal_scale = camera(base_rotation, base_translation, params)
    shifts = []
    for k, pixels in captures:
        start = table_points(pixels, k, base_rotation, base_translation, HEIGHT)
        fitted = table_points(pixels, k, rotation, translation, HEIGHT)
        shifts.append((start - fitted).mean(0))
    shift = np.mean(shifts, axis=0)
    translation = translation + [shift[0], shift[1], 0.0]
    print(
        f"anchored: moved camera x,y by "
        f"{shift[0] * 1000:+.1f}, {shift[1] * 1000:+.1f} mm"
    )
    out = dict(pose)
    out["rotation_matrix"] = [[float(v) for v in row] for row in rotation]
    out["translation_m"] = [float(v) for v in translation]
    out["quaternion_xyzw"] = [float(v) for v in quaternion_xyzw(rotation)]
    out["grid_fit"] = {
        "tilt_x_deg": float(np.degrees(params[0])),
        "tilt_y_deg": float(np.degrees(params[1])),
        "height_change_mm": float(params[2] * 1000),
        "placements": stems,
        "note": (
            "tilt and height fit to grids; camera x, y and yaw from the start "
            "pose, x, y re-anchored to the grids average position"
        ),
    }
    yaml.safe_dump(out, open(out_path, "w"))
    print("wrote", out_path)


if __name__ == "__main__":
    main()
