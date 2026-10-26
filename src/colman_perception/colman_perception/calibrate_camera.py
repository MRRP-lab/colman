"""
Calibrates the wrist camera against the tabletop.

Reads the views captured by calib_sweep and adjusts the camera's mount on the
hand and the table's position and size, so that the table edges seen in the color
image lie on the known edge lines in the base frame. The camera's own
intrinsics, from its factory calibration, are taken as correct. Depth is not used.

Usage: python calibrate_camera.py <view directory> [output.yaml] [--freeze a,b,...]
--freeze holds the named parameters (see NAMES) at their starting values.
"""

import glob
import os
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import yaml

# The tabletop in the base frame: x from NEAR to FAR, y from -HALF to +HALF.
NEAR_X_M = 0.09
FAR_X_M = 0.63
HALF_WIDTH_M = 0.5625



def table_edges(half_width, far_x):
    """
    The tabletop's edges in the table frame, as (point, direction, span axis,
    span range): y = +half_width, y = -half_width, x = far_x, x = NEAR_X_M.
    """
    return [
        (np.array([0.0, half_width]), np.array([1.0, 0.0]), 0, (NEAR_X_M, far_x)),
        (np.array([0.0, -half_width]), np.array([1.0, 0.0]), 0, (NEAR_X_M, far_x)),
        (np.array([far_x, 0.0]), np.array([0.0, 1.0]), 1, (-half_width, half_width)),
        (np.array([NEAR_X_M, 0.0]), np.array([0.0, 1.0]), 1, (-half_width, half_width)),
    ]


# The edges as measured by hand, in the base frame.
EDGES = table_edges(HALF_WIDTH_M, FAR_X_M)

# Measurement noise, used to weigh the edge points against the priors.
SIGMA_EDGE_M = 0.002

# Where to look for a known edge in the color image: this many pixels either
# side of where the factory calibration puts it, sampled every SAMPLE_M along it.
SEARCH_PX = 40
SAMPLE_M = 0.005

# A gradient must be at least this strong (gray levels per pixel) to count.
MIN_GRADIENT = 4.0

# The table's edge is the first strong step in the median brightness profile
# across the edge, going outward from the tabletop: a step at least this share
# of the strongest one. It is found separately in up to STEP_CHUNKS stretches of
# the edge, since the projected edge drifts from the real one along its length,
# and a line through those. Then each scan line's strongest gradient within
# PEAK_WINDOW_PX of that line is a point on the edge.
STEP_SHARE = 0.3
STEP_CHUNKS = 4
MIN_CHUNK_SCANS = 10
MAX_CHUNK_SPREAD_PX = 12
PEAK_WINDOW_PX = 6

# A found edge must be within this angle of the projected one, and its points
# within this many pixels of the line.
MAX_ANGLE_RAD = np.radians(5.0)
EDGE_FIT_PX = 2.0
LINE_ITERATIONS = 400

# A found edge needs this many scan lines on it, and this share of those searched.
MIN_EDGE_POINTS = 10
MIN_EDGE_SHARE = 0.2

EDGE_NAMES = ["y=+0.5625", "y=-0.5625", "x=0.63", "x=0.09"]

# Overlay colors (BGR).
COLOR_EDGE_COLOR = (255, 255, 0)
FACTORY_LINE_COLOR = (0, 0, 255)
FITTED_LINE_COLOR = (0, 220, 0)


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
        self.rgb = cv2.imread(path[:-4] + "_rgb.png")
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


def project(rotation, translation, k, points):
    """Color image pixels of base frame points, and their depth in the camera."""
    camera = (points - translation) @ rotation
    z = np.where(camera[:, 2] > 1e-6, camera[:, 2], np.nan)
    return (
        np.column_stack(
            [k[0, 0] * camera[:, 0] / z + k[0, 2], k[1, 1] * camera[:, 1] / z + k[1, 2]]
        ),
        camera[:, 2],
    )


def parabola_peak(values, i):
    """Position of the peak near index i, to a fraction of a sample."""
    a, b, c = values[i - 1], values[i], values[i + 1]
    denominator = a - 2 * b + c
    return i + (0.5 * (a - c) / denominator if denominator != 0 else 0.0)


def first_step(profile):
    """
    Index, to a fraction of a sample, of the first strong step in a profile,
    looking from its end inward to its start, or None.
    """
    gradient = np.abs(np.gradient(profile))
    threshold = max(MIN_GRADIENT, STEP_SHARE * gradient.max())
    for i in range(len(gradient) - 2, 1, -1):
        if (
            gradient[i] >= threshold
            and gradient[i] >= gradient[i - 1]
            and gradient[i] > gradient[i + 1]
        ):
            return parabola_peak(gradient, i)
    return None


def find_color_edge(gray, rotation, translation, k, edge):
    """
    Looks for one known table edge in the color image, near where the factory
    calibration projects it. Takes brightness profiles across it, finds the
    table's edge in their median (the first strong step going outward from the
    tabletop, so a rail or bevel beyond the edge isn't taken for it), then
    takes each scan line's gradient peak there and fits a line through them
    with RANSAC. Returns the points on the line, or None.
    """
    origin, direction, _, (low, high) = edge
    inward = np.array([-direction[1], direction[0]])
    if (np.array([0.36, 0.0]) - origin) @ inward < 0:
        inward = -inward
    s = np.arange(low, high, SAMPLE_M)
    xy = origin[None, :] + s[:, None] * direction[None, :]
    on_table = np.zeros((len(xy), 1))
    pixels, depth = project(rotation, translation, k, np.hstack([xy, on_table]))
    inside, _ = project(
        rotation, translation, k, np.hstack([xy + 0.02 * inward, on_table])
    )
    height, width = gray.shape
    offsets = np.arange(-SEARCH_PX, SEARCH_PX + 1)
    profiles = []
    searched = []
    normals = []
    for i in range(1, len(pixels) - 1):
        if depth[i] < 0.05 or not np.all(np.isfinite(pixels[i - 1 : i + 2])):
            continue
        if not np.all(np.isfinite(inside[i])):
            continue
        tangent = pixels[i + 1] - pixels[i - 1]
        length = np.linalg.norm(tangent)
        if length < 1e-6:
            continue
        normal = np.array([-tangent[1], tangent[0]]) / length
        if normal @ (inside[i] - pixels[i]) < 0:
            normal = -normal
        samples = pixels[i][None, :] + offsets[:, None] * normal[None, :]
        u, v = samples[:, 0], samples[:, 1]
        if u.min() < 3 or v.min() < 3 or u.max() > width - 4 or v.max() > height - 4:
            continue
        profiles.append(
            cv2.remap(
                gray,
                u.astype(np.float32)[:, None],
                v.astype(np.float32)[:, None],
                cv2.INTER_LINEAR,
            )[:, 0]
        )
        searched.append(pixels[i])
        normals.append(normal)
    if len(searched) < MIN_EDGE_POINTS:
        return None
    profiles = np.array(profiles)
    chunks = np.array_split(
        np.arange(len(profiles)),
        max(1, min(STEP_CHUNKS, len(profiles) // MIN_CHUNK_SCANS)),
    )
    found = [(c.mean(), first_step(np.median(profiles[c], axis=0))) for c in chunks]
    found = np.array([(c, st) for c, st in found if st is not None])
    if len(found) == 0:
        return None
    typical = np.median(found[:, 1])
    found = found[np.abs(found[:, 1] - typical) < MAX_CHUNK_SPREAD_PX]
    if len(found) >= 2:
        trend = np.polyfit(found[:, 0], found[:, 1], 1)
        steps = np.polyval(trend, np.arange(len(profiles)))
    else:
        steps = np.full(len(profiles), typical)

    candidates = []
    for scan, (profile, point, normal) in enumerate(zip(profiles, searched, normals)):
        gradient = np.abs(np.gradient(profile))
        window = np.arange(
            max(2, int(np.floor(steps[scan] - PEAK_WINDOW_PX))),
            min(len(gradient) - 2, int(np.ceil(steps[scan] + PEAK_WINDOW_PX))) + 1,
        )
        if len(window) == 0:
            continue
        best = window[np.argmax(gradient[window])]
        if gradient[best] < MIN_GRADIENT:
            continue
        candidates.append(
            (point + (parabola_peak(gradient, best) - SEARCH_PX) * normal, scan)
        )
    if len(candidates) < MIN_EDGE_POINTS:
        return None
    searched = np.array(searched)
    points = np.array([c[0] for c in candidates])
    scans = np.array([c[1] for c in candidates])
    # The projected edge's direction, to compare candidate lines to.
    axis = np.linalg.svd(searched - searched.mean(axis=0))[2][0]

    rng = np.random.default_rng(0)
    best_line = None
    for _ in range(LINE_ITERATIONS):
        a, b = points[rng.choice(len(points), 2, replace=False)]
        direction_px = b - a
        length = np.linalg.norm(direction_px)
        if length < 50.0:
            continue
        direction_px /= length
        if abs(direction_px[0] * axis[1] - direction_px[1] * axis[0]) > np.sin(
            MAX_ANGLE_RAD
        ):
            continue
        normal = np.array([-direction_px[1], direction_px[0]])
        count = int(np.sum(np.abs((points - a) @ normal) < EDGE_FIT_PX))
        if best_line is None or count > best_line[0]:
            best_line = (count, a, direction_px)
    if best_line is None:
        return None
    _, a, direction_px = best_line
    for _ in range(2):
        normal = np.array([-direction_px[1], direction_px[0]])
        inliers = np.abs((points - a) @ normal) < EDGE_FIT_PX
        if inliers.sum() < 2:
            return None
        a, direction_px = fit_line(points[inliers])
    normal = np.array([-direction_px[1], direction_px[0]])
    inliers = np.abs((points - a) @ normal) < EDGE_FIT_PX
    count = len(set(scans[inliers]))
    if count < MIN_EDGE_POINTS or count < MIN_EDGE_SHARE * len(searched):
        return None
    return points[inliers]


def ray_on_table(rotation, translation, k, pixels, height=0.0):
    """
    Where the rays through color image pixels meet the plane z = height, in the
    base frame.
    """
    rays = np.column_stack(
        [
            (pixels[:, 0] - k[0, 2]) / k[0, 0],
            (pixels[:, 1] - k[1, 2]) / k[1, 1],
            np.ones(len(pixels)),
        ]
    )
    direction = rays @ rotation.T
    along = (height - translation[2]) / direction[:, 2]
    return translation + direction * along[:, None], direction[:, 2]


def analyze(view):
    """
    Finds the table edge points in a view's color image, with the factory
    calibration, so the solver has fixed measurements to work with. Returns
    whether any edge was found.
    """
    rotation, translation = view.camera_pose()
    gray = cv2.GaussianBlur(cv2.cvtColor(view.rgb, cv2.COLOR_BGR2GRAY), (0, 0), 1.2)
    gray = gray.astype(np.float32)
    for index, edge in enumerate(EDGES):
        points = find_color_edge(gray, rotation, translation, view.rgb_k, edge)
        if points is None:
            continue
        view.edge_pixels.append(points)
        view.edge_ids.append(index)
    return bool(view.edge_ids)


NAMES = [
    "rx",
    "ry",
    "rz",
    "x",
    "y",
    "z",
    "table_x",
    "table_y",
    "table_yaw",
    "half_width",
    "far_x",
    "table_z",
]

# How far each parameter is expected to be from its starting value, in radians
# or meters. Mostly the edges decide; these only stop parameters the edges
# can't tell apart from running away.
PRIOR_SIGMA = np.array(
    [0.05, 0.05, 0.05, 0.03, 0.03, 0.03] + [0.002, 0.002, 0.004, 0.002, 0.002, 0.001]
)


def unpack(theta):
    return SimpleNamespace(
        correction=to_matrix(theta[0:3], theta[3:6]),
        table_offset=theta[6:8],
        yaw=theta[8],
        edges=table_edges(HALF_WIDTH_M + theta[9], FAR_X_M + theta[10]),
        height=theta[11],
    )


def table_to_base(model, xy):
    """Table frame points to the base frame, given the table's yaw and offset."""
    c, s = np.cos(model.yaw), np.sin(model.yaw)
    return xy @ np.array([[c, s], [-s, c]]) + model.table_offset


def edge_error(model, xy, index):
    """Signed distance of base frame points from a table edge, in meters."""
    c, s = np.cos(model.yaw), np.sin(model.yaw)
    in_table = (xy - model.table_offset) @ np.array([[c, -s], [s, c]])
    origin, direction, _, _ = model.edges[index]
    return (in_table - origin) @ np.array([-direction[1], direction[0]])


def edge_residuals(theta, views):
    """Distance of each color edge point from its known edge, in sigmas."""
    model = unpack(theta)
    edges = []
    for view in views:
        rotation, translation = view.camera_pose(model.correction)
        for pixels, index in zip(view.edge_pixels, view.edge_ids):
            xy, down = ray_on_table(
                rotation, translation, view.rgb_k, pixels, model.height
            )
            error = edge_error(model, xy[:, :2], index)
            error[down >= 0] = 1.0
            edges.append(error / SIGMA_EDGE_M)
    return np.concatenate(edges) if edges else np.zeros(0)


def residuals(theta, views, theta0):
    """
    The edge residuals plus priors that keep the parameters near their starting
    values, since the edges alone leave some combinations unconstrained.
    """
    return np.concatenate(
        [edge_residuals(theta, views), (theta - theta0) / PRIOR_SIGMA]
    )


def solve(views, theta0, frozen=()):
    """Fits every parameter except the frozen ones, which stay at theta0."""
    free = np.array([name not in frozen for name in NAMES])

    def fun(x, views, theta0):
        theta = theta0.copy()
        theta[free] = x
        return residuals(theta, views, theta0)

    theta = theta0.copy()
    theta[free] = least_squares(fun, theta0[free], args=(views, theta0))
    return theta


def rms_mm(theta, views):
    edge = edge_residuals(theta, views)
    return (
        float(np.sqrt(np.mean((edge * SIGMA_EDGE_M * 1000.0) ** 2)))
        if len(edge)
        else 0.0
    )


def view_edge_errors(view, theta):
    """
    {edge index: mean signed error in mm} for the edges found in one view.
    """
    model = unpack(theta)
    rotation, translation = view.camera_pose(model.correction)
    errors = {}
    for pixels, index in zip(view.edge_pixels, view.edge_ids):
        xy, _ = ray_on_table(rotation, translation, view.rgb_k, pixels, model.height)
        errors[index] = float(np.mean(edge_error(model, xy[:, :2], index)) * 1000.0)
    return errors


def known_edge_pixels(theta, view):
    """
    The four known table edges projected into the color image, as a list of
    runs of pixels (the parts in front of the camera).
    """
    model = unpack(theta)
    rotation, translation = view.camera_pose(model.correction)
    k = view.rgb_k
    runs = []
    for origin, direction, _, (low, high) in model.edges:
        s = np.linspace(low, high, 300)
        xy = table_to_base(model, origin[None, :] + s[:, None] * direction[None, :])
        points = np.column_stack([xy, np.full(len(xy), model.height)])
        camera = (points - translation) @ rotation
        z = np.maximum(camera[:, 2], 1e-6)
        uv = np.column_stack(
            [k[0, 0] * camera[:, 0] / z + k[0, 2], k[1, 1] * camera[:, 1] / z + k[1, 2]]
        )
        ok = (camera[:, 2] > 0.05) & (np.abs(uv).max(axis=1) < 20000)
        run = []
        for point, good in zip(uv, ok):
            if good:
                run.append(point)
            elif run:
                runs.append(np.array(run))
                run = []
        if run:
            runs.append(np.array(run))
    return runs


def draw_text(image, lines):
    scale = 0.9
    line_height = 36
    width = max(
        cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)[0][0]
        for text in lines
    )
    cv2.rectangle(
        image, (0, 0), (width + 20, line_height * len(lines) + 12), (0, 0, 0), -1
    )
    for i, text in enumerate(lines):
        cv2.putText(
            image,
            text,
            (10, line_height * (i + 1)),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )


def save_overlay(view, path, factory, fitted=None):
    """
    Saves the view's color image with what the calibration saw and used:
      cyan dots      table edge points found in the color image (what is fitted)
      red lines      the four known table edges projected with the factory
                     calibration
      green lines    the same with the fitted calibration
    factory and fitted are parameter vectors.
    """
    image = view.rgb.copy()
    for pixels in view.edge_pixels:
        for u, v in pixels:
            cv2.circle(image, (int(round(u)), int(round(v))), 5, COLOR_EDGE_COLOR, -1)
    lines = [view.name]
    for label, theta, color in (
        ("factory", factory, FACTORY_LINE_COLOR),
        ("fitted", fitted, FITTED_LINE_COLOR),
    ):
        if theta is None:
            continue
        for run in known_edge_pixels(theta, view):
            if len(run) > 1:
                cv2.polylines(
                    image,
                    [np.round(run).astype(np.int32)],
                    False,
                    color,
                    2,
                    cv2.LINE_AA,
                )
        errors = view_edge_errors(view, theta)
        text = ", ".join(f"{EDGE_NAMES[i]} {e:+.1f}" for i, e in sorted(errors.items()))
        lines.append(f"{label} (mm): {text or 'no edges'}")
    lines.append("cyan=edge points red=factory green=fitted")
    draw_text(image, lines)
    cv2.imwrite(path, image)


def edge_report(theta, views):
    """
    The signed error of each known edge, averaged over the views that saw it:
    positive means the edge is measured beyond where it really is.
    """
    errors = {i: [] for i in range(len(EDGES))}
    for view in views:
        for index, error in view_edge_errors(view, theta).items():
            errors[index].append(error)
    for index, values in errors.items():
        if values:
            print(
                f"  {EDGE_NAMES[index]:10s} mean {np.mean(values):+6.1f} mm, "
                f"spread {np.std(values):4.1f} mm over {len(values)} views"
            )


def main():
    args = sys.argv[1:]
    frozen = ()
    if "--freeze" in args:
        i = args.index("--freeze")
        frozen = tuple(args[i + 1].split(","))
        del args[i : i + 2]
        unknown = [name for name in frozen if name not in NAMES]
        if unknown:
            sys.exit(f"Unknown parameters {unknown}, choose from {NAMES}")
    folder = args[0]
    output = args[1] if len(args) > 1 else None
    paths = sorted(glob.glob(f"{folder}/view_??.npz"))
    if not paths:
        print("No views")
        return
    overlays = f"{folder}/overlays"
    os.makedirs(overlays, exist_ok=True)
    theta0 = np.zeros(len(NAMES))

    # Each view's overlay is written as soon as it is analyzed, so the
    # detections can be checked while the rest run.
    views = []
    every = []
    for path in paths:
        view = View(path)
        every.append(view)
        found = analyze(view)
        save_overlay(view, f"{overlays}/{view.name}_overlay.png", theta0)
        if found:
            views.append(view)
            print(f"{view.name}: edges {[EDGE_NAMES[i] for i in view.edge_ids]}")
        else:
            print(f"{view.name}: no table edges found")
    print(f"Overlays in {overlays}")

    def write_overlays(theta):
        for view in every:
            save_overlay(view, f"{overlays}/{view.name}_overlay.png", theta0, theta)
        print(f"\nFitted overlays in {overlays}")

    if len(views) < 4:
        print("Not enough views")
        return

    before = rms_mm(theta0, views)
    theta = solve(views, theta0, frozen)
    after = rms_mm(theta, views)
    print("\nEdge errors with the factory calibration:")
    edge_report(theta0, views)
    print("Edge errors after calibrating:")
    edge_report(theta, views)
    print(f"\nTable edge rms: {before:.1f} -> {after:.1f} mm")
    for name, a, b in zip(NAMES, theta0, theta):
        print(f"  {name:12s} {a:12.5f} -> {b:12.5f}")

    write_overlays(theta)

    # Cross-validation: fit on most views, test on the rest.
    folds = 4
    held_before, held_after = [], []
    for fold in range(folds):
        test = [v for i, v in enumerate(views) if i % folds == fold]
        train = [v for i, v in enumerate(views) if i % folds != fold]
        fitted = solve(train, theta0, frozen)
        held_before.append(rms_mm(theta0, test))
        held_after.append(rms_mm(fitted, test))
    held_before = float(np.mean(held_before))
    held_after = float(np.mean(held_after))
    print("\nHeld-out views (fit on the others):")
    print(f"  edge rms: {held_before:.1f} -> {held_after:.1f} mm")

    if output:
        with open(output, "w") as f:
            yaml.safe_dump(
                {
                    "parameters": {n: float(v) for n, v in zip(NAMES, theta)},
                    "initial": {n: float(v) for n, v in zip(NAMES, theta0)},
                    "edge_rms_mm": float(after),
                    "held_out_edge_rms_mm": held_after,
                },
                f,
            )
        print(f"Saved to {output}")


if __name__ == "__main__":
    main()
