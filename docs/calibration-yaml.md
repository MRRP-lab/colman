# How `calibration.yaml` is made

`calibration.yaml` is the output of `calibrate_camera.py`. It holds the fitted wrist-camera
mount (hand-eye) correction and the fitted tabletop pose and size. It lives next to the
views it was fitted to, e.g. `calib_data_20261026_1156/calibration.yaml`.

> **Which code?** The version of `calibrate_camera.py` that writes this file is on
> `origin/camera-calibration` (commit `836d076`, "fit hand-eye and table from color edges
> only", with `fb9d0d4` on top). An older version of the script (the one in commit `ebab6e0`)
> fits camera intrinsics and a depth scale and writes a different yaml. If your yaml has no
> `fx`/`depth_scale` keys and does have `table_yaw`, it came from the newer one. Make sure the
> local branch is up to date before running it.

## Pipeline

```
pixi run calib-sweep        →  $CALIB_DIR/view_NN.npz + view_NN_rgb.png   (24 views)
python calibrate_camera.py <view dir> <view dir>/calibration.yaml
```

1. **`calib_sweep.py`** moves the arm to 24 stationary views (6 at each table corner) and saves
   for each one: the colour image, the `base→tool` pose, the `tool→camera` pose (from the
   URDF) and the camera intrinsics.
2. **`calibrate_camera.py`** finds the tabletop edges in every colour image, then fits the
   parameters below so the edges land on the known edge lines. Depth is not used, and the
   camera's factory intrinsics are taken as correct.

## Finding the table edges in an image (`analyze`)

For each of the four edges (`y=+0.5625`, `y=-0.5625`, `x=0.63`, `x=0.09`):

1. Project the known edge into the colour image with the *factory* camera pose
   (`base_from_tool @ tool_from_camera`).
2. Take brightness profiles across it, ±40 px, every 5 mm along the edge.
3. The table's edge is the first strong step in the median profile going *outward from the
   tabletop*, so a rail or bevel beyond the edge isn't mistaken for it. It is found
   separately in up to 4 stretches, since the projected edge drifts from the real one along
   its length.
4. Each scan line's strongest gradient near that line is an edge point, to a fraction of a
   pixel. RANSAC fits a line through them; the edge is kept only if it is within 5° of the
   projected one and has enough points.

Edge points are fixed after this step. The solver does not re-detect them.

## What is fitted (`NAMES`)

| name | meaning |
|---|---|
| `rx ry rz` (rad), `x y z` (m) | hand-eye correction: `camera = base_from_tool @ tool_from_camera @ correction`, the rotation as a Rodrigues vector |
| `table_x`, `table_y` (m) | table frame origin in the base frame |
| `table_yaw` (rad) | table frame rotation about z |
| `half_width` (m) | added to 0.5625, the half-width in y |
| `far_x` (m) | added to 0.63, the far edge |
| `table_z` (m) | height of the table plane in the base frame |

The near edge (x = 0.09) is **not fitted**; it stays at the hand-measured value. (The older
depth-based detector never found it in `calib_data_20261026_1156`; I haven't checked whether the
newer colour-edge detector does.)

Conventions (these are what to use if you read the yaml yourself):

```
base_from_table = [ Rz(+table_yaw) | table_x, table_y, table_z ]
far edge   : x = 0.63   + far_x      (table frame)
side edges : y = ±(0.5625 + half_width)
near edge  : x = 0.09
```

The ray from each edge pixel is intersected with the plane `z = table_z`, moved into the
table frame, and its distance to the edge line is the residual (weighted by
`SIGMA_EDGE_M` = 2 mm). Gentle priors (about ±2 mm for the table terms, ±0.05 rad / ±3 cm for
the hand-eye) stop parameters the edges can't separate from running away, so the table offsets
are partly prior-driven, not fully measured.

The solver is a Levenberg–Marquardt with a soft-L1 loss (`least_squares` in the script), with
no scipy. `--freeze a,b,...` holds the named parameters at 0.

## What it writes

```yaml
parameters:           # the fit, in NAMES order
  rx: ... ry: ... rz: ...   x: ... y: ... z: ...
  table_x: ... table_y: ... table_yaw: ...
  half_width: ... far_x: ... table_z: ...
initial:              # all zeros (the starting point)
edge_rms_mm:          # residual on the views used to fit
held_out_edge_rms_mm: # 4-fold cross-validation: fit on 3/4 of the views, test on the rest
```

For `calib_data_20261026_1156`: `edge_rms_mm` 0.97, `held_out_edge_rms_mm` 1.09 (about 4.5 mm
before fitting).

It also writes an overlay PNG for every view into `<view dir>/overlays/`: red is the factory
projection of the known edges, green is the fitted one, and the dots are the detected edge
points. **This overwrites existing overlays in that folder.**

## Using it for the overhead camera

`calibrate_overhead.py <capture.npz> <capture_rgb.png> <out.yaml> <calibration.yaml>` reads
only the table terms (`table_x/y/z`, `table_yaw`, `half_width`, `far_x`) to build the four
edges in the base frame, finds the same four edges in the overhead image and solves the
overhead camera's pose. Without the last argument it uses the hand-measured table, which is about
4 mm too high and 0.2° off. The hand-eye terms are not used. The result is
`src/colman_perception/colman_perception/config/overhead_camera.yaml`.

Capture notes: the overhead camera autofocuses badly at 1.8 m. Fix the lens at manual
position 118 (sweep it with `CameraControl.setManualFocus` and take the sharpest, by Laplacian
variance). Stereo depth is useless at that range, so it isn't used.

## Gotcha: memory

An older `table_finder.py` computed `np.linalg.svd(points - centroid)` with full matrices on
about 44,000 points. That allocates a 44,000 × 44,000 array (14.6 GB) and was OOM-killing
sessions. It needs `full_matrices=False`. The `origin/camera-calibration` version no longer has
that file.
