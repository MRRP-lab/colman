# Camera calibration

This folder holds the official camera calibrations, the data they were fitted to, and
how to redo them. The tools themselves are in the ROS packages (see
[Where the code is](#where-the-code-is)).

## What is calibrated

Only each camera's **position**: where it is on the hand (wrist) or above the table
(overhead). Two things are taken as correct:

- **The arm.** Its forward kinematics is the ground truth (the UR3e's factory kinematic
  calibration, matched by `ur_config.yaml`).
- **The lenses.** The OAK-D Pro's factory intrinsics and on-device undistortion are used
  as they are. A fit that also allowed a focal-length change found 1.003, so there is no
  sign of a problem, but no checkerboard intrinsic calibration has been done.

## The official calibrations

| Camera | File | Used by | Error against the touched circles |
|---|---|---|---|
| Overhead camera pose | [`results/overhead_camera_pose.yaml`](results/overhead_camera_pose.yaml) | `src/colman_perception/.../config/overhead_camera.yaml` (the static transform) | **0.9 mm** on points not used in the fit |
| Wrist camera mount | [`results/wrist_camera_mount.yaml`](results/wrist_camera_mount.yaml) | `oakd_pro_joint` in `colman.urdf.xacro` | **median 0.8 mm**, rms 2.3 mm (1.5 mm without circle 1) |

The error is how far the camera puts a known point from where the arm's nozzle touched
it, in `base_link`. It is the only measurement tied to the arm, so it is the one to use
when comparing calibrations.

### How the alternatives scored

Same ground truth, total rms error in mm. Their result files are not kept here.

| Calibration | Overhead | Wrist |
|---|---|---|
| Touch-off fit (**official overhead**) | **0.9** | |
| Grid fit (**official wrist**) | | **2.3** |
| Hand-eye fit `c8a4f18` (was on `main`) | | 2.9 |
| Table edges + grid corners merged | 3.1 | |
| Table-edge fit | 5.2 | 6.9 (original mount) |
| Grid-only overhead fit | 5.2 | |

Two things that did not help: merging results from different calibrations (it blended
in the errors of the hand-measured references), and refitting the wrist mount jointly on
grid views and circle observations (held-out error got slightly worse).

## Layout

```
calibration/
  results/                       the two official result files (only these are kept)
  data/
    overhead_touch_off/          touched points (two sets), circle pixels, intrinsics
    wrist_grid/                  15 views of the grid on the table (corners, poses)
    wrist_circles/               15 wrist views of the touched circles
  docs/                          background and the older table-edge method
```

## Where the code is

| Package | Tools |
|---|---|
| `colman_perception` | `touch_off_pick`, `touch_off_record`, `touch_off_record_once`, `touch_off_fit`, `touch_off_check`, `grid_fit_wrist`, `wrist_circle_score`, `capture_grid` |
| `colman_motion` | `grid_sweep_wrist`, `wrist_circle_sweep` |
| older method | `calib_sweep`, `calibrate_camera`, `calibrate_overhead`, `grid_fit_overhead`, `grid_check_overhead`, `merge_overhead` |

All are `ros2 run <package> <tool>`; the arm tools need the driver and motion server (and
the wrist camera) running. Each tool prints its usage with no arguments.

## How to redo them

### Overhead camera (touch-off)

1. Draw marks on the table where the nozzle can reach (this set used 8 circles on tape).
2. Take an overhead image: `capture_grid`. Find the marks in it with `touch_off_pick`,
   which writes `pixels.yaml`.
3. Put the vacuum nozzle on each mark, by hand in freedrive, and record it:
   `touch_off_record_once touched.yaml <mark>`. Use the same light touch every time.
4. `touch_off_fit <capture stem> touched.yaml pixels.yaml <start pose> <out.yaml>`
5. Check it with `touch_off_check` (or `pixi run touch-off-check`) on a second set of
   touches that was not used for the fit.

### Wrist camera (grid)

1. Put a flat grid (7x5 inner corners, 31 mm squares here) on the table.
2. `grid_sweep_wrist <out dir> <grid x> <grid y>` moves the arm slowly through views and
   saves the corners and tool pose of each.
3. `grid_fit_wrist <view dir> <out.yaml>`

### Check a wrist mount against the touched circles

`wrist_circle_sweep <out dir>` takes views that each see several circles, and
`wrist_circle_score <view dir>` scores every mount in `wrist_circle_plan.MOUNTS` against
the touched positions. Add a mount there to compare it.

## Caveats

- **The nozzle offset is from the URDF** (`vacuum_tip_link`), not measured on the real
  nozzle. An error in it would look like a camera offset. The rubber tip also compresses
  by an unknown 0 to 2 mm, so touched heights are not the table height.
- **The table is not flat.** Touched heights show two shallow bowls either side of the
  support beam, about 3 mm deep. The fits use one plane at the mean touched height, which
  costs at most about 0.5 mm in x and y.
- **27 wrist observations** in 15 views is a small set. The worst errors come from steep
  views of the far circles (circles 1 and 6), where a small table-height error becomes a
  sideways one.
- **The 4K overhead capture is not stored** (14 MB). `touch_off_check --pixels` does not
  need it; retaking marks with `touch_off_pick` does.
- The data in `data/` was taken with the camera, arm and table as they were in
  November 2026. Redo the touch-off if the overhead camera or the arm base is moved.
