"""
Checks overhead camera poses against points the robot touched.

The robot's forward kinematics is far more accurate than anything measured by
hand, so a point the arm has touched is a ground truth position in base_link.
This script finds where the touched points are in an overhead image, projects
them onto the table with each camera pose under test, and reports how far the
projections are from the touched points. The points are either marks drawn on
the table where the tool touched it, or corners of a grid lying on the table.

    touch_off_check <capture stem> <touched.yaml> <pose.yaml> [<pose.yaml> ...]
        [--pixels <pixels.yaml>]

The stem has <stem>.npz and <stem>_rgb.png (from capture_grid.py).

Marks: find them in the image with touch_off_pick, which writes pixels.yaml and
numbers each mark, record the touched positions under the same numbers with
touch_off_record, and pass the pixels file with --pixels.

Grid: touch_off_record the corners, referring to each by its index in the
detected order, and leave out --pixels. Run

    touch_off_check --annotate <capture stem> <out.png> [cols rows]

first to get an image with every corner's index drawn on it.

For each pose it prints the error at every touched point (projected minus
touched, mm) and a summary:

    offset     mean error in x and y. A constant offset is a translation error.
    rms        error left after removing the offset.
    scale      size of the projected points against the touched ones about
               their centre. Not 1.0 means camera height or focal length is off.
    yaw        rotation between the two sets, in degrees.
    fit rms    error left after removing offset, scale and yaw. This is noise
               in the corners and in where the tip was placed.

Projection is onto the plane at the mean height of the touched points, so a
card or board under the grid does not need to be modelled.
"""

import sys

import cv2
import numpy as np
import yaml

COLS, ROWS = 7, 5


def find_corners(gray, cols=COLS, rows=ROWS):
    """
    Inner corners of the grid in a single image, or None. Falls back to
    overlapping windows about a third of the image wide, as the detector can
    miss a grid on a large, busy frame that it finds easily in a crop.
    """
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
    if ok:
        return corners.reshape(-1, 2)
    height, width = gray.shape
    size = max(width, height) // 3
    step = size // 4
    for y in range(0, max(height - size, 0) + 1, step):
        for x in range(0, max(width - size, 0) + 1, step):
            window = gray[y : y + size, x : x + size]
            ok, corners = cv2.findChessboardCornersSB(window, (cols, rows), flags)
            if ok:
                return corners.reshape(-1, 2) + [x, y]
    return None


def refine_mark(gray, x, y, radius=12):
    """
    Centre of a pen mark near a click, to a fraction of a pixel. Takes the
    darkness or brightness of the window's pixels against their median as
    weights and keeps only the strongest half, so the table texture does not
    drag the centre. Returns the click unchanged if there is no clear mark.
    """
    x0, y0 = int(round(x)), int(round(y))
    top, left = max(y0 - radius, 0), max(x0 - radius, 0)
    window = gray[top : y0 + radius + 1, left : x0 + radius + 1].astype(float)
    if window.size == 0:
        return float(x), float(y)
    diff = np.abs(window - np.median(window))
    if diff.max() < 20:
        return float(x), float(y)
    weights = np.where(diff >= diff.max() / 2, diff, 0.0)
    ys, xs = np.mgrid[0 : window.shape[0], 0 : window.shape[1]]
    total = weights.sum()
    return (
        float((weights * xs).sum() / total + left),
        float((weights * ys).sum() / total + top),
    )


def table_points(pixels, k, rotation, translation, height):
    """Pixels to base_link x, y on the plane z = height."""
    rays = (np.linalg.inv(k) @ np.c_[pixels, np.ones(len(pixels))].T).T @ rotation.T
    scale = (height - translation[2]) / rays[:, 2]
    return (translation + rays * scale[:, None])[:, :2]


def similarity_fit(touched, projected):
    """
    Scale, yaw (degrees) and residual rms (m) of the best rigid fit with scale
    taking the touched points onto the projected ones, about their centres.
    """
    a = touched - touched.mean(0)
    b = projected - projected.mean(0)
    u, s, vt = np.linalg.svd(b.T @ a)
    d = np.diag([1, np.sign(np.linalg.det(u @ vt))])
    rot = u @ d @ vt  # b ~ scale * a @ rot.T
    scale = (s * d.diagonal()).sum() / (a**2).sum()
    resid = b - scale * a @ rot.T
    yaw = np.degrees(np.arctan2(rot[1, 0], rot[0, 0]))
    return scale, yaw, np.sqrt((resid**2).sum(1).mean())


def compare_pose(pixels, k, pose, touched_xyz):
    """
    How far a camera pose's projection of the touched points' pixels is from
    where they were touched. pixels and touched_xyz are matched row for row.
    Returns a dict of errors in mm, projected minus touched.
    """
    touched_xyz = np.asarray(touched_xyz, float)
    rotation = np.asarray(pose["rotation_matrix"], float)
    translation = np.asarray(pose["translation_m"], float)
    height = touched_xyz[:, 2].mean()
    projected = table_points(pixels, k, rotation, translation, height)
    error = projected - touched_xyz[:, :2]
    offset = error.mean(0)
    scale, yaw, fit_rms = similarity_fit(touched_xyz[:, :2], projected)
    return {
        "height": height,
        "error_mm": error * 1000,
        "offset_mm": offset * 1000,
        "rms_mm": np.sqrt(((error - offset) ** 2).sum(1).mean()) * 1000,
        "total_rms_mm": np.sqrt((error**2).sum(1).mean()) * 1000,
        "scale": scale,
        "yaw_deg": yaw,
        "fit_rms_mm": fit_rms * 1000,
    }


def load_capture(stem):
    k = np.load(f"{stem}.npz")["rgb_k"]
    image = cv2.imread(f"{stem}_rgb.png")
    if image is None:
        sys.exit(f"cannot read {stem}_rgb.png")
    return k, image


def annotate(stem, out, cols, rows):
    _, image = load_capture(stem)
    pixels = find_corners(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cols, rows)
    if pixels is None:
        sys.exit("grid not found")
    for i, (x, y) in enumerate(pixels):
        cv2.circle(image, (int(x), int(y)), 4, (0, 0, 255), -1)
        cv2.putText(
            image,
            str(i),
            (int(x) + 5, int(y) - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )
    cv2.imwrite(out, image)
    print(f"wrote {out} with {len(pixels)} corners")


def point_id(point):
    """A touched point's label; older files call it the corner index."""
    return point["label"] if "label" in point else point["corner"]


def report(name, result, labels):
    print(f"\n{name}")
    for label, (ex, ey) in zip(labels, result["error_mm"]):
        print(f"  point {label!s:>4}  x {ex:+7.1f}  y {ey:+7.1f}")
    ox, oy = result["offset_mm"]
    print(f"  plane z = {result['height'] * 1000:.1f} mm")
    print(f"  offset     x {ox:+.1f}  y {oy:+.1f} mm")
    print(
        f"  rms        {result['rms_mm']:.1f} mm  (total {result['total_rms_mm']:.1f})"
    )
    print(f"  scale      {result['scale']:.4f}")
    print(f"  yaw        {result['yaw_deg']:+.2f} deg")
    print(f"  fit rms    {result['fit_rms_mm']:.1f} mm")


def main():
    args = sys.argv[1:]
    if args and args[0] == "--annotate":
        if len(args) < 3:
            sys.exit(__doc__)
        cols, rows = (int(args[3]), int(args[4])) if len(args) > 4 else (COLS, ROWS)
        annotate(args[1], args[2], cols, rows)
        return
    pixels_path = None
    if "--pixels" in args:
        i = args.index("--pixels")
        if i + 1 >= len(args):
            sys.exit(__doc__)
        pixels_path = args[i + 1]
        del args[i : i + 2]
    if len(args) < 3:
        sys.exit(__doc__)
    stem, touched_path, pose_paths = args[0], args[1], args[2:]

    touched = yaml.safe_load(open(touched_path))
    points = touched["points"]
    if len(points) < 3:
        sys.exit("need at least 3 touched points")
    labels = [point_id(p) for p in points]

    if pixels_path:
        k = np.load(f"{stem}.npz")["rgb_k"]
        marks = {
            str(m["label"]): m["pixel"]
            for m in yaml.safe_load(open(pixels_path))["points"]
        }
        missing = [str(label) for label in labels if str(label) not in marks]
        if missing:
            sys.exit(f"no pixel position for points {', '.join(missing)}")
        pixels = np.array([marks[str(label)] for label in labels], float)
    else:
        k, image = load_capture(stem)
        cols, rows = touched.get("grid", [COLS, ROWS])
        corners = find_corners(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cols, rows)
        if corners is None:
            sys.exit("grid not found")
        indices = [int(label) for label in labels]
        if max(indices) >= len(corners) or min(indices) < 0:
            sys.exit(f"corner index out of range, the grid has {len(corners)} corners")
        pixels = corners[indices]
    xyz = np.array([p["xyz"] for p in points], float)

    for path in pose_paths:
        pose = yaml.safe_load(open(path))
        report(path, compare_pose(pixels, k, pose, xyz), labels)


if __name__ == "__main__":
    main()
