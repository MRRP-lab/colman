# Camera pose calibration: where each camera is

This folder holds the fitted positions of the two cameras, the data they were fitted to,
and how to fit them again. The tools are in the ROS packages (see
[Tools](#tools)).

## What is fitted, and what is not

Two transforms, each fitted so that points the camera sees land where the arm's nozzle
actually touched them:

| Name | Meaning | Where it is used |
|---|---|---|
| **Wrist camera mount** | The fixed transform from `tool0` (the robot's tool flange) to the wrist camera's optical frame. Also called the hand-eye transform. | `oakd_pro_joint` in `colman.urdf.xacro` |
| **Overhead camera pose** | The fixed transform from `base_link` to the overhead camera's optical frame. | The static transform in `config/overhead_camera_pose_in_base_link.yaml` |

Two things are taken as correct and are **not** fitted here:

- **The arm.** Its forward kinematics is the ground truth. The UR3e's factory kinematic
  parameters are loaded from `ur_config.yaml` and the serial number and hash match.
- **Lens intrinsics** (focal length, principal point, distortion). The OAK-D Pro's
  factory intrinsics and its on-device undistortion are used as they are. A fit that also
  allowed a focal-length change found 1.003, so there is no sign of a problem, but the
  lenses have not been checked with a checkerboard.

## The fitted results in use

| Camera | File | Error against the touched points |
|---|---|---|
| Overhead camera pose in `base_link` | [`results/overhead_camera_pose_in_base_link.yaml`](results/overhead_camera_pose_in_base_link.yaml) | **0.9 mm** on points not used in the fit |
| Wrist camera mount on the tool flange | [`results/wrist_camera_mount_on_tool_flange.yaml`](results/wrist_camera_mount_on_tool_flange.yaml) | **median 0.8 mm**, rms 2.3 mm (1.5 mm without circle 1) |

The error is how far a camera puts a known point from where the arm's nozzle touched it,
in `base_link`. It is the only measurement tied to the arm, so it is the one to use when
comparing fits.

### How the other fits scored

Same points, total rms error in mm. Their result files are not kept here.

| Fit | Overhead camera pose | Wrist camera mount |
|---|---|---|
| Overhead pose fitted to touched points (**in use**) | **0.9** | |
| Wrist mount fitted to grid views (**in use**) | | **2.3** |
| Wrist mount from hand-eye samples (`c8a4f18`, previously on `main`) | | 2.9 |
| Overhead pose fitted to table edges and grid corners together | 3.1 | |
| Overhead pose fitted to table edges | 5.2 | |
| Overhead tilt and height fitted to grids | 5.2 | |
| Wrist mount before any fit (the CAD-nominal value) | | 6.9 |

Two things that did not help: merging fits made from different references (it blended in
their errors), and refitting the wrist mount jointly on grid views and circle
observations (error on held-out views got slightly worse).

## Layout

```
camera_pose_calibration/
  results/                                   the two fits in use
  data/
    overhead_pose_from_touched_points/       touched points (two sets), circle centres, intrinsics
    wrist_mount_from_grid_views/             15 views of a grid on the table (corners, tool pose)
    wrist_mount_check_with_touched_circle_views/   15 wrist views of the touched circles
  docs/                                      background, and the older table-edge method
```

## Tools

All are `ros2 run <package> <tool>`. The tools that move the arm need the driver, the
motion server and the wrist camera running. Each tool prints its usage when run with no
arguments.

**Fitting the overhead camera pose from touched points** (package `colman_perception`)

| Tool | What it does |
|---|---|
| `capture_undistorted_camera_still` | Saves one undistorted still and the camera's intrinsics |
| `click_touch_off_marks` | You click the marks drawn on the table in the still; writes their pixel centres |
| `record_touched_point_once` | Records where the nozzle is, under a mark's number (`record_touched_points` is the interactive version) |
| `fit_overhead_pose_to_touched_points` | Fits the overhead camera pose to the touched points |
| `check_overhead_pose_against_touched_points` | Scores one or more overhead poses against the touched points |

**Fitting the wrist camera mount from grid views**

| Tool | Package | What it does |
|---|---|---|
| `capture_wrist_grid_views` | `colman_motion` | Moves the arm through views of a grid on the table and saves the corners and tool pose |
| `fit_wrist_mount_to_grid_views` | `colman_perception` | Fits the wrist camera mount to those views |

**Checking a wrist camera mount against the touched circles**

| Tool | Package | What it does |
|---|---|---|
| `capture_wrist_views_of_touched_circles` | `colman_motion` | Takes wrist views that each see several touched circles |
| `score_wrist_mounts_against_touched_circles` | `colman_perception` | Scores every mount listed in `plan_wrist_views_of_touched_circles.MOUNTS` against the touched positions |

**Older table-edge method** (the fits it gave scored worse, so only its tools and docs are kept)

`capture_wrist_table_edge_views`, `fit_wrist_mount_to_table_edges`,
`fit_overhead_pose_to_table_edges`, `fit_overhead_tilt_and_height_to_grids`,
`check_overhead_pose_with_grid`, `merge_overhead_pose_fits`. The older hand-eye tools are
`record_wrist_hand_eye_samples` and `solve_wrist_mount_from_hand_eye_samples`.

## How to fit them again

### Overhead camera pose from touched points

1. Draw marks on the table where the nozzle can reach (this set used 8 circles on tape).
2. Take an overhead still with `capture_undistorted_camera_still`, then click the marks
   with `click_touch_off_marks`. It writes `mark_pixels.yaml`.
3. Put the vacuum nozzle on each mark, by hand in freedrive, and record it with
   `record_touched_point_once touched_points.yaml <mark number>`. Use the same light
   touch every time.
4. `fit_overhead_pose_to_touched_points <capture stem> touched_points.yaml
   mark_pixels.yaml <start pose> <out.yaml>`
5. Score it with `check_overhead_pose_against_touched_points` (or
   `pixi run check-overhead-pose-against-touched-points`) on a second set of touches that
   was not used in the fit.

### Wrist camera mount from grid views

1. Put a flat grid (7x5 inner corners, 31 mm squares here) on the table. Its rough
   position comes from `check_overhead_pose_with_grid`, which prints the grid's centre.
2. `capture_wrist_grid_views <out dir> <grid x> <grid y>`
3. `fit_wrist_mount_to_grid_views <view dir> <out.yaml>`

### Checking a wrist camera mount against the touched circles

`capture_wrist_views_of_touched_circles <out dir>`, then
`score_wrist_mounts_against_touched_circles <view dir>`. To compare another mount, add it
to `MOUNTS` in `plan_wrist_views_of_touched_circles.py`.

## Caveats

- **The nozzle offset is from the URDF** (`vacuum_tip_link`), not measured on the real
  nozzle. An error in it would look like a camera offset. The rubber tip also compresses
  by an unknown 0 to 2 mm, so touched heights are not the table height.
- **The table is not flat.** Touched heights show two shallow bowls either side of the
  support beam, about 3 mm deep. The fits use one plane at the mean touched height, which
  costs at most about 0.5 mm in x and y.
- **27 wrist observations** in 15 views is a small set. The worst errors come from steep
  views of the far circles (circles 1 and 6), where a small error in table height becomes
  a sideways one.
- **The 4K overhead still is not stored** (14 MB). Scoring with `--pixels` does not need
  it; clicking marks again does.
- **The data in `data/`** was taken with the cameras, arm and table as they were in
  November 2026. Fit again if the overhead camera or the arm's base is moved.
