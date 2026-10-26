"""
Calibrates the wrist camera against the tabletop.

Reads the views captured by calib_sweep and adjusts the camera's mount on the
hand, its intrinsics and a depth scale and offset, so that:
  - the table surface seen in depth lies at z = 0 in the base frame,
  - the table edges seen in the color image lie on the known edge lines.

Usage: python calibrate_camera.py <view directory> [output.yaml]
"""

import glob
import sys

import cv2
import numpy as np
import yaml

from colman_perception.table_finder import fit_table

# The tabletop in the base frame: x from NEAR to FAR, y from -HALF to +HALF.
NEAR_X_M = 0.09
FAR_X_M = 0.63
HALF_WIDTH_M = 0.5625

# Known edges as (point, direction, span axis, span range) in the base frame.
EDGES = [
    (np.array([0.0, HALF_WIDTH_M]), np.array([1.0, 0.0]), 0, (NEAR_X_M, FAR_X_M)),
    (np.array([0.0, -HALF_WIDTH_M]), np.array([1.0, 0.0]), 0, (NEAR_X_M, FAR_X_M)),
    (np.array([FAR_X_M, 0.0]), np.array([0.0, 1.0]), 1, (-HALF_WIDTH_M, HALF_WIDTH_M)),
    (np.array([NEAR_X_M, 0.0]), np.array([0.0, 1.0]), 1, (-HALF_WIDTH_M, HALF_WIDTH_M)),
]

# How far past an edge's ends a point may be and still belong to it.
SPAN_MARGIN_M = 0.10

# Points further than this from every known edge are not table edge.
MAX_EDGE_ERROR_M = 0.05

# Measurement noise, used to weigh the terms against each other.
SIGMA_PLANE_M = 0.003
SIGMA_EDGE_M = 0.002

PLANE_SAMPLES = 1500

# The depth image is this many times smaller than the color image.
DEPTH_SCALE_DOWN = 3.0

# How far from the depth-found edge to look for the real one in the color image,
# and the spacing of the points taken along it.
SEARCH_PX = 30
STEP_PX = 10

# A color edge point must be this close to the fitted line to be kept.
EDGE_FIT_PX = 2.5
MIN_EDGE_KEEP = 0.6

MIN_EDGE_POINTS = 30


def depth_intrinsics(k):
    """
    The depth camera's intrinsics, given the color camera's. Pixel centers sit
    at half-integer coordinates, so the principal point scales as
    (c + 0.5) / n - 0.5, not c / n.
    """
    kd = np.array(k, dtype=np.float64)
    kd[:2, :2] /= DEPTH_SCALE_DOWN
    kd[:2, 2] = (kd[:2, 2] + 0.5) / DEPTH_SCALE_DOWN - 0.5
    return kd


def depth_to_color_pixels(pixels):
    """Where depth image pixels fall in the color image."""
    return (np.asarray(pixels, dtype=np.float64) + 0.5) * DEPTH_SCALE_DOWN - 0.5


def to_matrix(rotation_vector, translation):
    m = np.eye(4)
    m[:3, :3] = cv2.Rodrigues(np.asarray(rotation_vector, dtype=np.float64))[0]
    m[:3, 3] = translation
    return m


def least_squares(fun, x0, args=(), f_scale=2.0, iterations=60):
    """
    Levenberg-Marquardt with a soft L1 loss, so a few bad edge points don't
    drag the answer. Returns the parameters.
    """

    def cost(r):
        return float(
            np.sum(2.0 * f_scale**2 * (np.sqrt(1.0 + (r / f_scale) ** 2) - 1.0))
        )

    x = np.array(x0, dtype=np.float64)
    r = fun(x, *args)
    best = cost(r)
    damping = 1e-2
    for _ in range(iterations):
        steps = 1e-6 * np.maximum(np.abs(x), 1e-3)
        jacobian = np.empty((len(r), len(x)))
        for i in range(len(x)):
            shifted = x.copy()
            shifted[i] += steps[i]
            jacobian[:, i] = (fun(shifted, *args) - r) / steps[i]
        weight = (1.0 + (r / f_scale) ** 2) ** -0.5
        jw = jacobian * weight[:, None]
        normal = jacobian.T @ jw
        gradient = jw.T @ r
        improved = False
        for _ in range(10):
            step = np.linalg.solve(
                normal + damping * np.diag(np.diag(normal)), -gradient
            )
            trial = fun(x + step, *args)
            trial_cost = cost(trial)
            if trial_cost < best:
                x, r, best = x + step, trial, trial_cost
                damping = max(damping / 3.0, 1e-9)
                improved = True
                break
            damping *= 4.0
        if not improved or np.linalg.norm(step) < 1e-10:
            break
    return x


class View:
    def __init__(self, path):
        data = np.load(path)
        self.name = path.split("/")[-1][:-4]
        self.base_from_tool = data["base_from_tool"]
        self.tool_from_camera = data["tool_from_camera"]
        self.rgb_k = data["rgb_k"]
        # The depth is aligned to the color camera and scaled down evenly, so use
        # that, not the depth intrinsics saved with the view, which are wrong.
        self.depth_k = depth_intrinsics(self.rgb_k)
        self.depth_mm = data["depth"].astype(np.float32)
        self.rgb = cv2.imread(path[:-4] + "_rgb.png")
        self.plane_pixels = None
        self.edge_pixels = []
        self.edge_ids = []

    def camera_pose(self, correction=np.eye(4)):
        m = self.base_from_tool @ self.tool_from_camera @ correction
        return m[:3, :3], m[:3, 3]


def fit_line(points):
    dx, dy, x0, y0 = cv2.fitLine(
        points.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01
    ).flatten()
    return np.array([x0, y0]), np.array([dx, dy])


def refine_edge_in_color(gray, depth_pixels):
    """
    Given the depth image pixels of an edge, finds the same edge in the color
    image to a fraction of a pixel, and returns points along it, or None.
    """
    color_pixels = depth_to_color_pixels(depth_pixels)
    point, direction = fit_line(color_pixels.astype(np.float32))
    normal = np.array([-direction[1], direction[0]])
    along = (color_pixels - point) @ direction
    found = []
    for s in np.arange(along.min(), along.max(), STEP_PX):
        offsets = np.arange(-SEARCH_PX, SEARCH_PX + 1)
        samples = (point + s * direction)[None, :] + offsets[:, None] * normal[None, :]
        u, v = samples[:, 0], samples[:, 1]
        if (
            u.min() < 2
            or v.min() < 2
            or u.max() > gray.shape[1] - 3
            or v.max() > gray.shape[0] - 3
        ):
            continue
        profile = cv2.remap(
            gray,
            u.astype(np.float32)[:, None],
            v.astype(np.float32)[:, None],
            cv2.INTER_LINEAR,
        )[:, 0]
        gradient = np.abs(np.gradient(profile))
        best = int(np.argmax(gradient))
        if best < 2 or best > len(gradient) - 3 or gradient[best] < 3.0:
            continue
        # Parabola through the peak for the position to a fraction of a pixel.
        a, b, c = gradient[best - 1], gradient[best], gradient[best + 1]
        shift = 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) != 0 else 0.0
        found.append(samples[best] + shift * normal)
    if len(found) < MIN_EDGE_POINTS // 3:
        return None
    found = np.array(found)
    point, direction = fit_line(found)
    normal = np.array([-direction[1], direction[0]])
    keep = np.abs((found - point) @ normal) < EDGE_FIT_PX
    if keep.mean() < MIN_EDGE_KEEP:
        return None
    return found[keep]


def assign_edge(points_xy):
    """
    The known edge a set of base frame points most likely belongs to, or None.
    """
    best, best_error = None, MAX_EDGE_ERROR_M
    for index, (origin, direction, axis, (low, high)) in enumerate(EDGES):
        normal = np.array([-direction[1], direction[0]])
        error = np.median(np.abs((points_xy - origin) @ normal))
        along = points_xy[:, axis]
        if along.min() < low - SPAN_MARGIN_M or along.max() > high + SPAN_MARGIN_M:
            continue
        if error < best_error:
            best, best_error = index, error
    return best


def ray_on_table(rotation, translation, k, pixels):
    rays = np.column_stack(
        [
            (pixels[:, 0] - k[0, 2]) / k[0, 0],
            (pixels[:, 1] - k[1, 2]) / k[1, 1],
            np.ones(len(pixels)),
        ]
    )
    direction = rays @ rotation.T
    scale = -translation[2] / direction[:, 2]
    return translation + direction * scale[:, None], direction[:, 2]


def analyze(view):
    """
    Finds the table pixels and edge points in a view, with the factory
    calibration, so the solver has fixed measurements to work with.
    """
    rotation, translation = view.camera_pose()
    result = fit_table(view.depth_mm / 1000.0, view.depth_k, rotation, translation)
    if result is None:
        return False
    # Table pixels, the ones on the plane found in depth, so cubes and the like
    # don't count.
    u, v = np.meshgrid(np.arange(640), np.arange(360))
    z = view.depth_mm / 1000.0
    kd = view.depth_k
    points = np.dstack(
        [(u - kd[0, 2]) / kd[0, 0] * z, (v - kd[1, 2]) / kd[1, 1] * z, z]
    )
    points = points @ rotation.T + translation
    normal = np.array(result["normal"])
    distance = points @ normal - result["height_m"] * normal[2]
    inside = (result["mask"] > 0) & (view.depth_mm > 0) & (np.abs(distance) < 0.015)
    ids = np.flatnonzero(inside.ravel())
    ids = np.random.default_rng(0).choice(
        ids, min(PLANE_SAMPLES, len(ids)), replace=False
    )
    view.plane_pixels = np.column_stack(
        [u.ravel()[ids], v.ravel()[ids], view.depth_mm.ravel()[ids]]
    )
    gray = cv2.GaussianBlur(cv2.cvtColor(view.rgb, cv2.COLOR_BGR2GRAY), (0, 0), 1.2)
    gray = gray.astype(np.float32)
    for edge in result["edges"]:
        points = refine_edge_in_color(gray, edge["pixels"])
        if points is None:
            continue
        xy, _ = ray_on_table(rotation, translation, view.rgb_k, points)
        index = assign_edge(xy[:, :2])
        if index is None:
            continue
        view.edge_pixels.append(points)
        view.edge_ids.append(index)
    return True


def unpack(theta):
    correction = to_matrix(theta[0:3], theta[3:6])
    fx, fy, cx, cy = theta[6:10]
    return correction, (fx, fy, cx, cy), theta[10], theta[11]


def residuals(theta, views, theta0, split=None):
    correction, (fx, fy, cx, cy), depth_scale, depth_offset = unpack(theta)
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    kd = depth_intrinsics(k)
    planes, edges = [], []
    for view in views:
        rotation, translation = view.camera_pose(correction)
        if view.plane_pixels is not None and len(view.plane_pixels):
            u, v, d = view.plane_pixels.T
            z = (depth_scale * d + depth_offset) / 1000.0
            points = np.column_stack(
                [(u - kd[0, 2]) / kd[0, 0] * z, (v - kd[1, 2]) / kd[1, 1] * z, z]
            )
            planes.append(((points @ rotation.T + translation)[:, 2]) / SIGMA_PLANE_M)
        for pixels, index in zip(view.edge_pixels, view.edge_ids):
            xy, down = ray_on_table(rotation, translation, k, pixels)
            origin, direction, _, _ = EDGES[index]
            normal = np.array([-direction[1], direction[0]])
            error = (xy[:, :2] - origin) @ normal
            error[down >= 0] = 1.0
            edges.append(error / SIGMA_EDGE_M)
    prior = np.concatenate(
        [
            theta[0:3] / 0.05,
            theta[3:6] / 0.03,
            [
                (fx - theta0[6]) / (0.03 * theta0[6]),
                (fy - theta0[7]) / (0.03 * theta0[7]),
            ],
            [(cx - theta0[8]) / 15.0, (cy - theta0[9]) / 15.0],
            [(depth_scale - 1.0) / 0.05, depth_offset / 30.0],
        ]
    )
    plane = np.concatenate(planes) if planes else np.zeros(0)
    edge = np.concatenate(edges) if edges else np.zeros(0)
    if split is not None:
        return plane, edge
    return np.concatenate([plane, edge, prior])


def solve(views, theta0):
    return least_squares(residuals, theta0, args=(views, theta0))


def rms_mm(theta, views, theta0):
    plane, edge = residuals(theta, views, theta0, split=True)
    return (
        float(np.sqrt(np.mean((plane * SIGMA_PLANE_M * 1000.0) ** 2)))
        if len(plane)
        else 0.0,
        float(np.sqrt(np.mean((edge * SIGMA_EDGE_M * 1000.0) ** 2)))
        if len(edge)
        else 0.0,
    )


def edge_report(theta, views):
    """
    The signed error of each known edge, averaged over the views that saw it:
    positive means the edge is measured beyond where it really is.
    """
    correction, (fx, fy, cx, cy), _, _ = unpack(theta)
    k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    names = ["y=+0.5625", "y=-0.5625", "x=0.63", "x=0.09"]
    errors = {i: [] for i in range(len(EDGES))}
    for view in views:
        rotation, translation = view.camera_pose(correction)
        for pixels, index in zip(view.edge_pixels, view.edge_ids):
            xy, _ = ray_on_table(rotation, translation, k, pixels)
            origin, direction, _, _ = EDGES[index]
            normal = np.array([-direction[1], direction[0]])
            errors[index].append(np.mean((xy[:, :2] - origin) @ normal) * 1000.0)
    for index, values in errors.items():
        if values:
            print(
                f"  {names[index]:10s} mean {np.mean(values):+6.1f} mm, "
                f"spread {np.std(values):4.1f} mm over {len(values)} views"
            )


def main():
    folder = sys.argv[1]
    output = sys.argv[2] if len(sys.argv) > 2 else None
    paths = sorted(glob.glob(f"{folder}/view_??.npz"))
    views = []
    for path in paths:
        view = View(path)
        if analyze(view):
            views.append(view)
            print(
                f"{view.name}: {len(view.plane_pixels)} plane pixels, "
                f"{len(view.edge_pixels)} edges {view.edge_ids}"
            )
        else:
            print(f"{view.name}: no tabletop found")
    if len(views) < 4:
        print("Not enough views")
        return

    k = views[0].rgb_k
    theta0 = np.array([0, 0, 0, 0, 0, 0, k[0, 0], k[1, 1], k[0, 2], k[1, 2], 1.0, 0.0])
    names = [
        "rx",
        "ry",
        "rz",
        "x",
        "y",
        "z",
        "fx",
        "fy",
        "cx",
        "cy",
        "depth_scale",
        "depth_offset_mm",
    ]

    before = rms_mm(theta0, views, theta0)
    theta = solve(views, theta0)
    after = rms_mm(theta, views, theta0)
    print("\nEdge errors with the factory calibration:")
    edge_report(theta0, views)
    print("Edge errors after calibrating:")
    edge_report(theta, views)
    print(f"\nTable plane rms: {before[0]:.1f} -> {after[0]:.1f} mm")
    print(f"Table edge rms:  {before[1]:.1f} -> {after[1]:.1f} mm")
    for name, a, b in zip(names, theta0, theta):
        print(f"  {name:16s} {a:12.5f} -> {b:12.5f}")

    # Cross-validation: fit on most views, test on the rest.
    folds = 4
    held_before, held_after = [], []
    for fold in range(folds):
        test = [v for i, v in enumerate(views) if i % folds == fold]
        train = [v for i, v in enumerate(views) if i % folds != fold]
        fitted = solve(train, theta0)
        held_before.append(rms_mm(theta0, test, theta0))
        held_after.append(rms_mm(fitted, test, theta0))
    held_before = np.mean(held_before, axis=0)
    held_after = np.mean(held_after, axis=0)
    print("\nHeld-out views (fit on the others):")
    print(f"  plane rms: {held_before[0]:.1f} -> {held_after[0]:.1f} mm")
    print(f"  edge rms:  {held_before[1]:.1f} -> {held_after[1]:.1f} mm")

    if output:
        with open(output, "w") as f:
            yaml.safe_dump(
                {
                    "parameters": {n: float(v) for n, v in zip(names, theta)},
                    "initial": {n: float(v) for n, v in zip(names, theta0)},
                    "rms_mm": {"plane": after[0], "edge": after[1]},
                    "held_out_rms_mm": {
                        "plane": float(held_after[0]),
                        "edge": float(held_after[1]),
                    },
                },
                f,
            )
        print(f"Saved to {output}")


if __name__ == "__main__":
    main()
