"""
Checks the overhead camera pose against a flat grid lying on the table.

Finds the grid's inner corners, projects them onto the table plane (z = 0)
with the overhead camera pose, and fits an ideal grid to them. The fitted
scale tells whether height/focal length are right (1.0 is right) and the
RMS residual is how far the corners are from a perfect flat grid.

Usage:
    python grid_check_overhead.py <capture.npz> <capture_rgb.png> \
        <overhead_camera.yaml> [inner corners across] [inner corners down] [square mm]
"""

import sys

import cv2
import numpy as np
import yaml


def find_corners(gray, cols, rows):
    """
    Inner corners of the grid, or None. On large, busy images the detector can
    miss a grid it finds easily in a crop, so it falls back to overlapping
    windows about a quarter of the image wide.
    """
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
    if ok:
        return corners.reshape(-1, 2)
    height, width = gray.shape
    size = max(width, height) // 4
    for y in range(0, max(height - size, 0) + 1, size // 2):
        for x in range(0, max(width - size, 0) + 1, size // 2):
            window = gray[y : y + size, x : x + size]
            ok, corners = cv2.findChessboardCornersSB(window, (cols, rows), flags)
            if ok:
                return corners.reshape(-1, 2) + [x, y]
    return None


def table_points(pixels, k, rotation, translation):
    """Pixels to base_link x, y on the plane z = 0."""
    rays = (np.linalg.inv(k) @ np.c_[pixels, np.ones(len(pixels))].T).T @ rotation.T
    scale = -translation[2] / rays[:, 2]
    return (translation + rays * scale[:, None])[:, :2]


def fit_grid(measured, model):
    """Best rigid fit with scale, allowing a mirrored grid. Returns scale, rms mm, yaw."""
    best = None
    for mirror in (1, -1):
        b = model * [1, mirror]
        a0, b0 = measured - measured.mean(0), b - b.mean(0)
        u, s, vt = np.linalg.svd(b0.T @ a0)
        rot = u @ vt  # row vectors: measured ~ scale * model @ rot
        if np.linalg.det(rot) < 0:
            continue
        scale = s.sum() / (b0**2).sum()
        rms = np.sqrt(((a0 - scale * b0 @ rot) ** 2).sum(1).mean()) * 1000
        yaw = np.degrees(np.arctan2(rot[0, 1], rot[0, 0]))
        if best is None or rms < best[1]:
            best = (scale, rms, yaw)
    return best


def main():
    capture, image_path, pose_path = sys.argv[1:4]
    cols, rows = (int(sys.argv[4]), int(sys.argv[5])) if len(sys.argv) > 5 else (7, 5)
    square = float(sys.argv[6]) / 1000 if len(sys.argv) > 6 else 0.031

    pose = yaml.safe_load(open(pose_path))
    rotation = np.array(pose["rotation_matrix"])
    translation = np.array(pose["translation_m"])
    k = np.load(capture)["rgb_k"]

    gray = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2GRAY)
    pixels = find_corners(gray, cols, rows)
    if pixels is None:
        sys.exit("grid not found")

    xy = table_points(pixels, k, rotation, translation)
    model = np.array([[i, j] for j in range(rows) for i in range(cols)], float) * square
    scale, rms, yaw = fit_grid(xy, model)
    spacing = np.linalg.norm(np.diff(xy.reshape(rows, cols, 2), axis=1), axis=2).mean()
    print(f"corners {len(pixels)}  centre {xy.mean(0).round(4)} m  yaw {yaw:.1f} deg")
    print(f"scale {scale:.4f}  spacing {spacing * 1000:.2f} mm  rms {rms:.2f} mm")


if __name__ == "__main__":
    main()
