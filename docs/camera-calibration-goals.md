# Camera calibration: goals

## The problem

The wrist camera (OAK-D Pro, mounted on the UR3e hand) is meant to tell the arm where
things are on the table: AprilTag cubes now, objects to pick up later. Positions computed
from the camera and then expressed in `base_link` are currently off by roughly **1 cm**
from where the things really are. That is too much to pick anything up reliably.

We already correct for lens distortion (the publisher asks the OAK for undistorted frames),
and we believe the arm's pose is good. So there is some remaining error whose source we
have not identified. See [camera-calibration-error-sources.md](camera-calibration-error-sources.md).

## Goal

Get camera-derived positions in `base_link` accurate to **~0.1 mm (ideal)**, from ~10 mm today.

0.1 mm may not be physically reachable with this hardware (see the error budget doc), so we
also want to know *which* error sources set the floor. A useful intermediate result is
"we are at ~1 mm and the remainder is X". Knowing where the error comes from matters as
much as the final number.

## Approach

Use the table itself as ground truth, and fit the unknown camera parameters so that what
the camera sees agrees with it.

Ground truth we have (measured by hand, see the memory/coordinate docs):

- Tabletop is 112.5 cm in y (centred on `base_link`, y = ±0.5625 m) by 54 cm in x
  (x = 0.09 m to 0.63 m). Its surface is z = 0.
- Four AprilTag cubes (tags 1, 3, 5, 6), 50 mm cubes about 1–2 cm inside the corners.
  The black tag square is 39.2 mm, not 40.
- The arm is UR3e serial 20245300947. Its kinematic calibration was pulled from the
  controller and the hash matches `ur_config.yaml`.

Unknowns we fit (`calibrate_camera.py`):

- the camera's mount on the hand (hand-eye transform),
- camera intrinsics,
- a depth scale and offset.

Constraints used to fit them, from views taken at many arm poses:

1. The table surface seen in depth must lie at z = 0 in `base_link`.
2. The table edges seen in the colour image must lie on the known edge lines.
3. (Cross-check) The tag positions seen from different views should be consistent
   with each other and with the cube geometry.

The corners are the most useful feature because an edge pair pins down position along
both x and y, and the far corners are viewed at the extreme of the arm's reach, which is
where kinematic error would show up most.

## Data collection

- `pixi run calib-sweep` (`calib_sweep.py`): 24 stationary views, 6 at each table corner
  (2 heights, 3 look targets). Saves colour image, median depth, base→tool pose,
  tool→camera pose and both intrinsics per view to `$CALIB_DIR`.
- `calibrate_camera.py <view dir> [out.yaml]` fits the camera parameters to those views, using
  `table_finder.py` to find the table plane and edges in depth.
- `pixi run find-table` / `pixi run view-depth`: live table finder and depth viewer.
- `camera_publisher.py` publishes aligned depth (640×360, mm) alongside the colour image,
  with the IR dot projector on so the black tabletop has texture for stereo.

## Success criteria

- Tag positions seen by the camera agree with hand-measured cube positions to <0.5 mm
  (first target), then <0.1 mm.
- Table plane from depth lies at z = 0 within <0.5 mm, with no systematic tilt.
- Table edges re-projected from the fitted model land on the detected edges within
  ~1 px in colour (note: 1 px at 0.3 m is roughly 0.2 mm with this camera at 1080p, so
  sub-pixel edge detection is needed to reach 0.1 mm).
- **Held-out views**: calibration is fit on a subset of the 24 views and evaluated on the
  rest. Residuals on views not used for the fit are the number that counts.
- Repeatability: the same pose revisited gives the same answer within the target.

## Open questions

- Is the hand-measured ground truth itself good to 0.1 mm? The tape-measure table
  dimensions probably are not. We may need to treat the table size as unknown too and
  rely on internal consistency, or measure with calipers.
- How much of the 1 cm is depth (stereo) vs. geometry (hand-eye, intrinsics, arm)? The
  experiments in the error doc are meant to answer this.
- Is the UR's reported pose accurate to better than ~0.1 mm at 0.38 m radius? UR3e
  repeatability is about ±0.03 mm, but *absolute* accuracy with factory calibration is
  not something we have verified.
