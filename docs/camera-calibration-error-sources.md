# Camera calibration: sources of error

Companion to [camera-calibration-goals.md](camera-calibration-goals.md). Observed error is
about 1 cm; the goal is ~0.1 mm. Everything below is a hypothesis unless marked
**measured**. The "rough size" figures are order-of-magnitude estimates from general
knowledge of this class of hardware, not measurements from our setup.

## Summary

| Source | Status | Rough size at the camera | Likely a main culprit? |
|---|---|---|---|
| Lens intrinsics / residual distortion | Corrected, never verified | 0.1–several mm | Possible |
| Hand-eye transform (camera on hand) | Nominal from URDF, never calibrated | mm to cm | **Very likely** |
| Stereo depth scale/offset | Uncalibrated | mm to cm at 0.3–0.4 m | **Very likely** |
| Arm absolute accuracy | Factory cal loaded, unverified | ~0.1–1 mm | Unlikely to explain 1 cm |
| Tag size / placement | Size measured, placement approximate | 0.1–1 mm | Contributes to floor |
| Tag pose estimation (PnP) | Not characterised | 0.1–1 mm | Contributes |
| Timing / motion | Mitigated by settling | small if still | Unlikely |
| Table ground truth | Tape-measured | ~0.5–1 mm | Limits the floor |

The key point: a 1 cm error is far larger than anything the arm or the tags should
contribute. It is almost certainly a **camera-side systematic** (hand-eye, intrinsics, depth),
not noise. Noise is already small: `tag_map.yaml` records per-tag scatter of 0.03–0.6 mm
(**measured**, std over 24–309 samples), so the camera is repeatable but not accurate.
Repeatable-but-wrong is the signature of a systematic offset.

## Lens distortion and intrinsics

- The OAK is asked for undistorted frames (`enableUndistortion=True`) and the published
  `camera_info` has zero distortion. So distortion is corrected by the device using
  factory calibration stored on it.
- Unknown: how good that factory calibration is. Factory intrinsics are typically good to
  a fraction of a percent in focal length, and the principal point can be off by several
  pixels. At 0.3 m with fx ≈ 1500 px (rough), a 1% focal length error is about 3 mm of
  lateral error at a point 0.3 m from the optical axis.
- Residual distortion is largest at the image corners, which is exactly where we put
  the table corners. This would show up as error that grows with distance from the
  image centre.
- The depth camera's reported vertical focal length was found to be **wrong** (see comment
  in `camera_publisher.py`), so the device's reported intrinsics are not entirely
  trustworthy. We replaced the depth intrinsics with the scaled colour intrinsics.
- Undistortion is done on a 1920×1080 output; the sensor's native aspect may be cropped or
  scaled, which affects how intrinsics map. Worth confirming `camera_info` matches the
  actual frames.

## Hand-eye transform (camera in tool frame)

- The camera pose relative to the hand comes from the URDF (`CAMERA_FRAME` relative to
  `tool0` / `vacuum_tip_link`). It is a CAD-nominal number, never calibrated.
- A mounting error of 1–2 mm in translation or 1° in rotation is entirely plausible
  (printed bracket, tolerances, sensor position inside the housing, and the optical
  centre not being where CAD says).
- Rotation error is amplified by distance: 0.5° at 0.4 m range is about 3.5 mm. A rotation
  error tilts the whole view, which would look like a position-dependent offset
  that changes sign when the hand rotates.
- Best single candidate for most of the 1 cm. Also the thing `calibrate_camera.py` fits.

## Arm kinematics (UR3e)

- We loaded the arm's own factory kinematic calibration, and the serial/hash match. This
  corrects link-length and joint-offset deviations the factory measured.
- Rated repeatability is about ±0.03 mm, but absolute accuracy depends on the calibration
  being right and on payload and temperature effects. We have no independent
  measurement of it. Plausible absolute error: ~0.1 to 1 mm, growing with reach.
- Joint compliance and payload sag: the camera mount adds payload, and the arm is
  stretched out near its reach when viewing far corners (about 0.38 m radius, with the
  camera held forward of the hand). Gravity-induced deflection could be tenths of
  a mm.
- We read the pose from TF (`/joint_states` through the URDF), not the controller's
  own TCP readout. The two should agree; comparing them would check the kinematics model.
- `tool0` vs `vacuum_tip_link`: `calib_sweep` uses `tool0` while the click/tag sweeps use
  `vacuum_tip_link` for the camera relation. Fine as long as each is consistent, but
  worth keeping in mind when comparing results.

## Tags: size, placement, pose estimation

- Tag size is **measured**: 39.2 mm black square, not 40 mm. The earlier 39 mm in the
  config was wrong by 0.2 mm; 1% tag size error is a 1% range error (about 3 mm at 0.3 m) so
  this matters, and it still needs checking with calipers or a scan at high resolution.
- The large tag (id 20) is configured at 98 mm and has not been verified.
- Printed tag scale: printers scale. The printed size may differ from the file.
- Tags sit on the **tops of 50 mm cubes**, so their z is about 50 mm in nominal terms, but
  `tag_map.yaml` has z of 37.5–44.7 mm (**measured**). That 6–12 mm discrepancy is
  suspicious: either the cubes are not exactly 50 mm, tag z estimates are biased
  (range/depth error), or the table is not at z=0 in the base frame. It is a useful
  clue: it is about the size of the 1 cm error, and varies from tag to tag (tag 5, with
  the fewest samples, is highest).
- PnP pose from a tag only 39 mm wide at 0.3–0.4 m is noisier in range and tilt than in
  the image plane. Corner detection sub-pixel accuracy is perhaps 0.1–0.3 px.
- Cube placement (1–2 cm in from the corner, hand-placed) is not known to 0.1 mm, so
  tags are poor as absolute ground truth for position; they are good for consistency checks.

## Depth / stereo

- Stereo depth is the weakest sensor here at close range. Error grows roughly with
  distance squared; at 0.3–0.4 m a typical OAK-D Pro is a few mm at best.
- Pipeline settings: depth is 640×360 aligned to the colour camera, extended disparity,
  subpixel, left-right check. Extended disparity is needed at short range but costs
  precision.
- A plain black tabletop is hard for stereo; the IR dot projector (intensity 0.6)
  helps, but black surfaces absorb IR, so texture is weak. This could produce biased
  or noisy depth on the table.
- Systematic scale/offset in depth is likely, which is why `calibrate_camera.py` fits
  a depth scale and offset. Median over 10 frames removes noise but not bias.
- Depth is aligned to the colour camera, so depth error couples to the intrinsics and
  extrinsics between the stereo cameras and the RGB camera.

## Timing, TF and motion

- `calib_sweep` settles for 1.5 s and rejects the view if the tool drifts over 0.5 mm during
  capture, so motion blur and drift should be small.
- Images are stamped on arrival in `camera_publisher` (`get_clock().now()`), not at exposure.
  Any latency is ignored. Harmless if the arm is stationary (as in calibration) but
  would matter for anything captured while moving.
- `Feed.pose` and `Recorder.pose` look up TF at `Time()` (latest), not at the image
  stamp. Again fine when stationary.
- Joint state lag when the arm has "stopped" could leave TF slightly behind the real
  pose; settle time should cover this.

## Table ground truth

- Table dimensions were measured with a tape: 112.5 cm × 54 cm. Probably good to
  ~1 mm, which is 10× worse than the target.
- The user noted the x orientation might be reversed. Measured edges at x ≈ 0.63 agree,
  but the position of the table relative to `base_link` is itself a measurement.
- Is the tabletop flat and at exactly z = 0? Is `base_link` really where the model
  says it is relative to the table? A 1 mm offset here is folded into every fit.
- If this is the floor, then calibration should aim at *internal consistency* (many
  views agreeing with each other and with a plane) rather than agreement with a tape
  measure.

## Experiments to separate the sources

Ordered by how cheaply they split the problem:

1. **Pose-independence of tag position.** Look at one fixed tag from many hand poses and
   rotations. If the estimated position in `base_link` scatters by mm, the error depends on
   pose, which points at hand-eye or intrinsics. If it is a constant offset, the
   error is probably in tag size or depth.
2. **Roll the camera about its own axis.** Rotate the hand about the optical axis while
   looking at the same tag. Hand-eye rotation error shows up as a circular drift of
   the estimated position.
3. **Distance sweep.** Move the camera straight toward/away from a tag. Error proportional to
   range points at tag size or focal length; constant error points at translation.
4. **Check intrinsics independently.** Calibrate with a printed checkerboard or ChArUco
   board (flat, rigid) using OpenCV, and compare against the OAK's factory values.
5. **Check the arm independently.** Touch a known point on the table with the tool
   tip from several approach directions, or compare TF pose against the controller's TCP
   pose, to bound arm error.
6. **Check depth against a flat reference.** Fit a plane to the table in depth at several
   ranges and compare against the tag-derived plane (depth vs. colour consistency).
7. **Split the data.** Fit on half of the 24 calibration views and evaluate on the other
   half; if fit residuals are small but held-out residuals are large, the model is
   absorbing noise rather than fixing a systematic.

## What to record as we learn

For each source, fill in an actual measured number here once we have it, and turn the
"rough size" column in the summary into measurements. Anything we cannot get below
0.1 mm sets the real floor for the whole system.
