"""
Finds where the fixed overhead camera is, in the base frame.

Uses the same idea as the wrist calibration: the tabletop's four edges are
known lines in base_link (z = 0). Each is found in the color image to a
fraction of a pixel, and the camera pose is solved so the known lines project
onto the detected ones.

Usage:
    python calibrate_overhead.py <capture.npz> <capture_rgb.png> <output.yaml> \
        [wrist calibration.yaml]

The capture holds the color intrinsics (rgb_k) of an undistorted color image,
and is made by a still, in-focus frame of the whole table.
"""

import sys

import cv2
import numpy as np
import yaml

from colman_perception.calibrate_camera import (
    EDGE_NAMES,
    EDGES,
    least_squares,
    to_matrix,
)

# Rough table corners in the image, in pixels (top-left, top-right,
# bottom-right, bottom-left), only to say where to start looking for each
# edge. The far edge (x = 0.63) is at the top of the image and +y is to the
# left.
SEED_CORNERS = np.array([[580, 209], [1482, 210], [1470, 646], [583, 640]], float)

# Which seed corners each known edge (in EDGES order: y=+, y=-, x=far, x=near)
# runs between.
SEED_EDGES = {0: (0, 3), 1: (1, 2), 2: (0, 1), 3: (3, 2)}

# The table surface is at z = 0.
SEARCH_PX = 25
FINE_SEARCH_PX = 6
STEP_PX = 4
INLIER_PX = 1.2
# Ends of an edge are left out, where corners are rounded and cluttered.
TRIM_FRACTION = 0.04
MIN_GRADIENT = 4.0


def fit_line(points, weights=None):
    dx, dy, x0, y0 = cv2.fitLine(
        points.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01
    ).flatten()
    return np.array([x0, y0]), np.array([dx, dy])


def edge_points(gray, point, direction, length, center, search):
    """
    Strongest brightness step across a line, sampled along it, to a fraction
    of a pixel. center is the middle of the stretch to sample.
    """
    normal = np.array([-direction[1], direction[0]])
    found = []
    half = length * (0.5 - TRIM_FRACTION)
    for s in np.arange(-half, half, STEP_PX):
        base = center + s * direction
        offsets = np.arange(-search, search + 1, dtype=np.float64)
        samples = base[None, :] + offsets[:, None] * normal[None, :]
        u, v = samples[:, 0], samples[:, 1]
        if (
            u.min() < 3
            or v.min() < 3
            or u.max() > gray.shape[1] - 4
            or v.max() > gray.shape[0] - 4
        ):
            continue
        profile = cv2.remap(
            gray,
            u.astype(np.float32)[:, None],
            v.astype(np.float32)[:, None],
            cv2.INTER_LINEAR,
        )[:, 0]
        gradient = np.gradient(profile)
        best = int(np.argmax(np.abs(gradient)))
        if best < 2 or best > len(gradient) - 3 or abs(gradient[best]) < MIN_GRADIENT:
            continue
        a, b, c = np.abs(gradient[best - 1 : best + 2])
        denominator = a - 2 * b + c
        shift = 0.5 * (a - c) / denominator if denominator != 0 else 0.0
        found.append((samples[best] + shift * normal, np.sign(gradient[best])))
    if not found:
        return np.zeros((0, 2)), np.zeros(0)
    return np.array([f[0] for f in found]), np.array([f[1] for f in found])


def ransac_line(points, rng, inlier=INLIER_PX, iterations=300):
    best = None
    for _ in range(iterations):
        i, j = rng.choice(len(points), 2, replace=False)
        d = points[j] - points[i]
        n = np.linalg.norm(d)
        if n < 50:
            continue
        d /= n
        normal = np.array([-d[1], d[0]])
        keep = np.abs((points - points[i]) @ normal) < inlier
        if best is None or keep.sum() > best.sum():
            best = keep
    return best


def find_edge(gray, seed_a, seed_b, rng):
    """Two passes: a wide search around the seed, then a narrow one."""
    seed_direction = seed_b - seed_a
    length = np.linalg.norm(seed_direction)
    seed_direction /= length
    center = (seed_a + seed_b) / 2
    points, signs = edge_points(gray, None, seed_direction, length, center, SEARCH_PX)
    if len(points) < 20:
        return None
    # Edges are all one polarity along a line; keep the commonest.
    points = points[signs == np.sign(np.sum(signs))]
    keep = ransac_line(points, rng, inlier=2.5)
    point, direction = fit_line(points[keep])
    if direction @ seed_direction < 0:
        direction = -direction
    # Narrow pass around the fitted line.
    center = point + ((center - point) @ direction) * direction
    points, signs = edge_points(gray, None, direction, length, center, FINE_SEARCH_PX)
    if len(points) < 20:
        return None
    points = points[signs == np.sign(np.sum(signs))]
    keep = ransac_line(points, rng)
    points = points[keep]
    point, direction = fit_line(points)
    normal = np.array([-direction[1], direction[0]])
    residual = (points - point) @ normal
    return {
        "points": points,
        "rms_px": float(np.sqrt(np.mean(residual**2))),
        "span_px": float(np.ptp((points - point) @ direction)),
    }


def project(k, rotation, translation, points):
    camera = (points - translation) @ rotation
    return np.column_stack(
        [
            k[0, 0] * camera[:, 0] / camera[:, 2] + k[0, 2],
            k[1, 1] * camera[:, 1] / camera[:, 2] + k[1, 2],
        ]
    )


def table_edges(calibration_path=None):
    """
    The four table edges as (end, end) pairs of base frame points, in EDGES
    order (y=+, y=-, x=far, x=near). Without a calibration they are the
    hand-measured edges at z = 0. With one (from the wrist fit), the table
    has its own pose in the base frame and the far edge and half width are
    adjusted:  base_from_table = [Rz(table_yaw) | table_x, table_y, table_z].
    The near edge was never seen by the wrist camera and stays at 0.09 m.
    """
    p = dict.fromkeys(
        ("table_x", "table_y", "table_z", "table_yaw", "far_x", "half_width"), 0.0
    )
    if calibration_path:
        with open(calibration_path) as f:
            p.update(yaml.safe_load(f)["parameters"])
    far = EDGES[2][0][0] + p["far_x"]
    near = EDGES[3][0][0]
    half = EDGES[0][0][1] + p["half_width"]
    cy, sy = np.cos(p["table_yaw"]), np.sin(p["table_yaw"])
    base_from_table = np.array(
        [
            [cy, -sy, 0.0, p["table_x"]],
            [sy, cy, 0.0, p["table_y"]],
            [0.0, 0.0, 1.0, p["table_z"]],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    def to_base(x, y):
        return (base_from_table @ np.array([x, y, 0.0, 1.0]))[:3]

    return [
        (to_base(far, half), to_base(near, half)),  # y = +half
        (to_base(far, -half), to_base(near, -half)),  # y = -half
        (to_base(far, half), to_base(far, -half)),  # x = far
        (to_base(near, half), to_base(near, -half)),  # x = near
    ]


def pose_from_theta(theta):
    """base_from_camera rotation and translation."""
    m = to_matrix(theta[:3], theta[3:6])
    return m[:3, :3], m[:3, 3]


def edge_residuals(theta, k, detected, edges):
    """Pixel distance of each detected point from its known edge's projection."""
    rotation, translation = pose_from_theta(theta)
    out = []
    for index, pixels in detected.items():
        a, b = project(k, rotation, translation, np.array(edges[index]))
        line = b - a
        line /= np.linalg.norm(line)
        normal = np.array([-line[1], line[0]])
        out.append((pixels - a) @ normal)
    return np.concatenate(out)


def corner_pose(k, edges):
    """First guess from the four corners."""
    # Corners in the base frame, in the order of SEED_CORNERS: where the far
    # edge (2) meets the +y edge (0) and the -y edge (1), then the near edge (3).
    corners = np.array(
        [edges[2][0], edges[2][1], edges[3][1], edges[3][0]], dtype=np.float64
    )
    # The planar solver wants the plane at z = 0; shift it there and back.
    z0 = float(np.mean(corners[:, 2]))
    corners = corners - np.array([0.0, 0.0, z0])
    ok, rvec, tvec = cv2.solvePnP(
        corners, SEED_CORNERS, k, None, flags=cv2.SOLVEPNP_IPPE
    )
    rotation = cv2.Rodrigues(rvec)[0]
    # solvePnP gives camera_from_base; invert.
    base_from_camera = np.eye(4)
    base_from_camera[:3, :3] = rotation.T
    base_from_camera[:3, 3] = -rotation.T @ tvec.ravel() + np.array([0.0, 0.0, z0])
    rv = cv2.Rodrigues(base_from_camera[:3, :3])[0].ravel()
    return np.concatenate([rv, base_from_camera[:3, 3]])


def rotation_to_quaternion(r):
    """x, y, z, w."""
    from math import copysign, sqrt

    t = np.trace(r)
    if t > 0:
        s = sqrt(t + 1.0) * 2
        return [
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
            0.25 * s,
        ]
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = sqrt(r[i, i] - r[j, j] - r[k, k] + 1.0) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return [copysign(1, q[3]) * v for v in q]


def draw_overlay(image, k, theta, detected, edges, path):
    rotation, translation = pose_from_theta(theta)
    for index, pixels in detected.items():
        for u, v in pixels:
            cv2.circle(image, (int(round(u)), int(round(v))), 3, (255, 255, 0), -1)
    for a, b in edges:
        s = np.linspace(0.0, 1.0, 200)[:, None]
        pts = np.array(a)[None, :] * (1 - s) + np.array(b)[None, :] * s
        uv = project(k, rotation, translation, pts)
        cv2.polylines(image, [np.round(uv).astype(np.int32)], False, (0, 220, 0), 1)
    cv2.imwrite(path, image)


def main():
    capture, image_path, output = sys.argv[1:4]
    calibration = sys.argv[4] if len(sys.argv) > 4 else None
    edges = table_edges(calibration)
    data = np.load(capture)
    k = data["rgb_k"]
    mxid = str(data["mxid"]) if "mxid" in data else None
    image = cv2.imread(image_path)
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (0, 0), 1.0)
    gray = gray.astype(np.float32)
    rng = np.random.default_rng(0)

    detected = {}
    for index, (a, b) in SEED_EDGES.items():
        found = find_edge(gray, SEED_CORNERS[a], SEED_CORNERS[b], rng)
        if found is None:
            print(f"{EDGE_NAMES[index]}: not found")
            continue
        detected[index] = found["points"]
        print(
            f"{EDGE_NAMES[index]:10s} {len(found['points'])} points, "
            f"span {found['span_px']:.0f} px, line rms {found['rms_px']:.2f} px"
        )
    if len(detected) < 3:
        print("Need at least three edges")
        return

    theta0 = corner_pose(k, edges)
    theta = least_squares(
        lambda t: edge_residuals(t, k, detected, edges), theta0, f_scale=1.0
    )
    r = edge_residuals(theta, k, detected, edges)
    print(f"\nEdge residual rms {np.sqrt(np.mean(r**2)):.2f} px")

    rotation, translation = pose_from_theta(theta)
    # Where one pixel lands on the table, to say what the residual means.
    mm_per_px = translation[2] / k[0, 0] * 1000.0
    print(f"1 px = {mm_per_px:.2f} mm on the table")
    print(f"camera position {np.round(translation, 4)} m (base_link)")
    quaternion = rotation_to_quaternion(rotation)
    print(f"quaternion xyzw {np.round(quaternion, 5)}")

    draw_overlay(image.copy(), k, theta, detected, edges, output[:-5] + "_overlay.png")
    with open(output, "w") as f:
        yaml.safe_dump(
            {
                "device_mxid": mxid,
                "parent_frame": "base_link",
                "child_frame": "overhead_camera_optical_frame",
                "translation_m": [float(v) for v in translation],
                "quaternion_xyzw": [float(v) for v in quaternion],
                "rotation_matrix": rotation.tolist(),
                "intrinsics": {
                    "fx": float(k[0, 0]),
                    "fy": float(k[1, 1]),
                    "cx": float(k[0, 2]),
                    "cy": float(k[1, 2]),
                    "width": image.shape[1],
                    "height": image.shape[0],
                },
                "static_transform_args": [
                    *(f"{v:.5f}" for v in translation),
                    *(f"{v:.5f}" for v in quaternion),
                    "base_link",
                    "overhead_camera_optical_frame",
                ],
                "table_model": {
                    "source": calibration or "hand-measured, nominal",
                    "far_edge_x_m": float(edges[2][0][0]),
                    "half_width_m": float(edges[0][0][1]),
                    "note": "near edge x=0.09 never seen by the wrist camera",
                },
                "fit": {
                    "edge_rms_px": float(np.sqrt(np.mean(r**2))),
                    "mm_per_px": float(mm_per_px),
                },
            },
            f,
        )
    print(f"Saved to {output}")


if __name__ == "__main__":
    main()
