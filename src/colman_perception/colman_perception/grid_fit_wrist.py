"""
Fits the wrist camera's mount on the hand (tool0 -> camera) to views of a flat
grid lying on the table, taken by grid_sweep_wrist.py.

The grid's own pose in the base frame is also fitted, because it only has to
be roughly known to take the views. Intrinsics are held at the camera's own
values (they are in each view). The fit minimises the grid corners'
reprojection error over all views, starting from the mount in the URDF.

Usage:
    python grid_fit_wrist.py <view dir> <out.yaml> [--intrinsics]
--intrinsics also fits a common focal scale and principal point shift.
--flat-grid forces the grid flat at the card's height (BOARD_M above z = 0) and fits
only its x, y and yaw. The wrist views do not agree with that unless z = 0 is the
table surface, so by default the grid's pose is free and its height is reported.
"""

import glob
import sys

import cv2
import numpy as np
import yaml

from colman_perception.calibrate_camera import least_squares

COLS, ROWS, SQUARE = 7, 5, 0.031
BOARD_M = 0.0047  # the grid card's thickness: its corners are this far above the table
URDF_TOOL_CAMERA = (0.0003, -0.1007, 0.044)  # tool0 -> camera, as in the URDF
MODEL = (
    np.array([[i, j, 0.0] for j in range(ROWS) for i in range(COLS)], float) * SQUARE
)


def pose(vector):
    m = np.eye(4)
    m[:3, :3] = cv2.Rodrigues(np.asarray(vector[:3], float))[0]
    m[:3, 3] = vector[3:]
    return m


def project(k, camera_from_world, points):
    p = (camera_from_world[:3, :3] @ points.T).T + camera_from_world[:3, 3]
    uv = (k @ p.T).T
    return uv[:, :2] / uv[:, 2:3]


def load(directory):
    views = []
    for path in sorted(glob.glob(f"{directory}/view_*.npz")):
        d = np.load(path)
        views.append((path, d["corners"], d["k"], d["base_tool"]))
    return views


def grid_in_base(corners, k, base_tool, mount):
    """The grid's pose in the base from one view, by PnP, for either corner order."""
    out = []
    for flip in (False, True):
        c = corners[::-1] if flip else corners
        ok, rvec, tvec = cv2.solvePnP(MODEL, c, k, None, flags=cv2.SOLVEPNP_IPPE)
        cam_grid = pose(np.r_[rvec.ravel(), tvec.ravel()])
        out.append((flip, base_tool @ mount @ cam_grid))
    return out


def main():
    views = load(sys.argv[1])
    mount0 = np.eye(4)
    mount0[:3, 3] = URDF_TOOL_CAMERA
    # the reference view fixes the corner order of the others
    ref_grid = grid_in_base(views[0][1], views[0][2], views[0][3], mount0)[0][1]
    ordered = []
    for path, corners, k, base_tool in views:
        options = grid_in_base(corners, k, base_tool, mount0)
        flip, grid = min(
            options,
            key=lambda o: (
                np.linalg.norm(o[1][:3, 3] - ref_grid[:3, 3])
                + 0.1 * np.linalg.norm(o[1][:3, :3] - ref_grid[:3, :3])
            ),
        )
        ordered.append((path, corners[::-1] if flip else corners, k, base_tool, flip))
    print(
        "views:",
        len(ordered),
        "flipped corner order in",
        sum(o[4] for o in ordered),
    )

    fit_intrinsics = "--intrinsics" in sys.argv
    free_grid = "--flat-grid" not in sys.argv

    # The grid lies flat on the table at the card's height, so unless --free-grid it
    # has only x, y and yaw. PnP puts the grid's z axis into the table (down), so a
    # flat grid is a turn about vertical, possibly after a half turn about x.
    flat = np.eye(3) if ref_grid[2, 2] > 0 else np.diag([1.0, -1.0, -1.0])
    n_grid = 6 if free_grid else 3
    n_mount = 6
    intrinsics_at = n_mount + n_grid

    def grid_matrix(x):
        g = x[n_mount:intrinsics_at]
        if free_grid:
            return pose(g)
        c, s = np.cos(g[0]), np.sin(g[0])
        turn = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        m = np.eye(4)
        m[:3, :3] = turn @ flat
        m[:3, 3] = [g[1], g[2], BOARD_M]
        return m

    if free_grid:
        grid0 = np.r_[cv2.Rodrigues(ref_grid[:3, :3])[0].ravel(), ref_grid[:3, 3]]
    else:
        turn0 = ref_grid[:3, :3] @ flat.T
        grid0 = np.r_[np.arctan2(turn0[1, 0], turn0[0, 0]), ref_grid[:2, 3]]
    x0 = np.r_[np.zeros(3), URDF_TOOL_CAMERA, grid0]
    if fit_intrinsics:
        x0 = np.r_[x0, 1.0, 0.0, 0.0]  # focal scale, principal point shift (px)

    def residuals(x, subset=None):
        mount = pose(x[:6])
        grid = grid_matrix(x)
        out = []
        for n, (_, corners, k, base_tool, _) in enumerate(ordered):
            if subset is not None and n not in subset:
                continue
            if fit_intrinsics:
                k = k.copy()
                k[0, 0] *= x[intrinsics_at]
                k[1, 1] *= x[intrinsics_at]
                k[0, 2] += x[intrinsics_at + 1]
                k[1, 2] += x[intrinsics_at + 2]
            cam_from_world = np.linalg.inv(base_tool @ mount)
            world = (grid[:3, :3] @ MODEL.T).T + grid[:3, 3]
            out.append((project(k, cam_from_world, world) - corners).ravel())
        return np.concatenate(out)

    def rms(r):
        return np.sqrt((r.reshape(-1, 2) ** 2).sum(1).mean())

    r0 = residuals(x0)
    fit_x = least_squares(residuals, x0, f_scale=1.0, iterations=300)
    r1 = residuals(fit_x)
    print(f"reprojection rms: URDF mount {rms(r0):.2f} px -> fitted {rms(r1):.2f} px")

    # per-view and leave-one-out
    n_view = len(ordered)
    per = r1.reshape(n_view, -1)
    print("per-view rms px:", np.round(np.sqrt((per**2).mean(1) * 2), 2).tolist())
    loo = []
    for held in range(n_view):
        keep = set(range(n_view)) - {held}
        x = least_squares(residuals, fit_x, args=(keep,), f_scale=1.0, iterations=100)
        loo.append(x[:6])
    loo = np.array(loo)
    print("leave-one-out spread of the fitted mount (mm, mrad):")
    print("  translation mm:", np.round(loo[:, 3:].std(0) * 1000, 2).tolist())
    print("  rotation mrad:", np.round(loo[:, :3].std(0) * 1000, 2).tolist())

    mount = pose(fit_x[:6])
    d_t = (mount[:3, 3] - np.array(URDF_TOOL_CAMERA)) * 1000
    angle = np.degrees(np.linalg.norm(fit_x[:3]))
    print(
        "mount change from URDF: translation "
        f"{np.round(d_t, 2).tolist()} mm, rotation {angle:.3f} deg"
    )
    grid = grid_matrix(fit_x)
    if fit_intrinsics:
        print(
            f"intrinsics: focal scale {fit_x[intrinsics_at]:.5f}, "
            "principal point shift "
            f"{fit_x[intrinsics_at + 1]:.2f}, {fit_x[intrinsics_at + 2]:.2f} px"
        )
    normal = grid[:3, :3] @ np.array([0, 0, 1.0])
    print(
        f"grid in base: origin {np.round(grid[:3, 3], 4).tolist()} m, tilt from flat "
        f"{np.degrees(np.arccos(abs(normal[2]))):.2f} deg"
    )
    print(
        f"implied table surface: z = {(grid[2, 3] - BOARD_M) * 1000:.1f} mm "
        "in base_link "
        f"(grid corners {grid[2, 3] * 1000:.1f} mm, card {BOARD_M * 1000:.1f} mm)"
    )
    rot = mount[:3, :3]
    w = np.sqrt(max(1 + rot[0, 0] + rot[1, 1] + rot[2, 2], 1e-12)) / 2
    q = [
        (rot[2, 1] - rot[1, 2]) / (4 * w),
        (rot[0, 2] - rot[2, 0]) / (4 * w),
        (rot[1, 0] - rot[0, 1]) / (4 * w),
        w,
    ]
    yaml.safe_dump(
        {
            "parent_frame": "tool0",
            "child_frame": "oakd_pro_camera_rgb_optical_frame_wrist",
            "translation_m": [float(v) for v in mount[:3, 3]],
            "quaternion_xyzw": [float(v) for v in q],
            "reprojection_rms_px": float(rms(r1)),
            "urdf_reprojection_rms_px": float(rms(r0)),
            "views": n_view,
            "method": "grid corner reprojection, grid pose fitted, intrinsics fixed",
        },
        open(sys.argv[2], "w"),
    )


if __name__ == "__main__":
    main()
