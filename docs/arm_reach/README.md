# Where the arm can reach on the table

![Arm reach on the table](arm_reach_on_table.jpg)

The picture is an overhead 4K still with the reach drawn on the table plane. The grid is every 10 cm in `base_link`: x is forward from the arm base and y is to the arm's left (left of the picture).

| Colour | Meaning |
| --- | --- |
| Green | The arm can pick a cube whose tag is on top of a 4-cube stack (tag z about 0.150 m). |
| Amber | The arm can pick a cube on the table (tag z about 0.033 m), but not from the top of a tall stack. |
| Red | The arm cannot reach, with the nozzle straight down. |

## Limits on the table (x from 0.09 to 0.63 m, y within +-0.565 m)

| | Cube on the table | Top of a 4-cube stack |
| --- | --- | --- |
| Furthest forward, y = 0 | x = 0.53 m | x = 0.47 m |
| Furthest forward, y = +0.2 | 0.47 m | 0.39 m |
| Furthest forward, y = -0.2 | 0.50 m | 0.44 m |
| Furthest left (+y), x = 0.25 | y = +0.42 m | y = +0.34 m |
| Furthest right (-y), x = 0.25 | y = -0.42 m | y = -0.42 m |
| Nearest to the base, y = 0 | x = 0.14 m | x = 0.14 m |

- Under half the table is reachable: 52% for a cube on the table, 39% for the top of a 4-stack.
- Nothing at x = 0.55 m or further, and nothing beyond about |y| 0.45 m.
- The far third of the table (x above 0.5 m) and both far side strips can't be used.
- Cubes should sit roughly within x 0.15 to 0.45 m and y -0.3 to +0.3 m. The tall stack is safest at x 0.15 to 0.35 m.
- A strip around the base (about 0.14 m in front of it at y = 0) is also unreachable.
- Reach is asymmetric: the arm reaches further left (+y) near the base and further forward on the right, because the nozzle yaw is fixed and wrist 3 is limited to -130..+140 degrees.
- The calibration marks #1 (y +0.51) and #8 (y -0.51) are outside the green area. They were touched with a free tool orientation, not with the nozzle straight down.

## How it was computed

- **Arm:** the URDF chain from `base_link` to `vacuum_tip_link`, with the kinematic parameters in `ur_config.yaml` and the joint limits in `colman_moveit_config/config/joint_limits.yaml` as on `main` after #44.
- **Pose:** a pick is the nozzle straight down (`EE_DOWN`, quaternion (1, 0, 0, 0)) at the tag z minus 0.01 m. The approach is 0.10 m above the tag, on the same IK solution. The table counts as reachable at a point only if both poses are found. Each point was tried from 24 starting joint configurations with a numerical 6D solver (position within 1 mm, orientation within 0.01 rad).
- **Heights:** the table plane is at z = -0.0066 m, from the touch-off points. A cube is 39.2 mm.
- **Projection:** the overhead pose in `camera_pose_calibration/results/overhead_camera_pose_in_base_link.yaml`, with the 4K intrinsics from `overhead_capture.npz`. Re-projecting the eight touched points lands within about 2 px of their marked centres.
- **Still:** `colman_calibration_archive/overhead_4k_capture/overhead_rgb.png`.

## What this does not include

- It is kinematics only. MoveIt collision checks, the table and the held cube's planning box, the wrist camera's view and motion-planner failures are not modelled, so the real usable area can be somewhat smaller. It does not include the place poses (0.05 m approach), only the pick poses.
- The solver is not TRAC-IK, so points near the green edge may flip either way. Treat the outline as +-1 to 2 cm.
- Only these two heights were tested. A stack of 2 or 3 cubes falls between the green and amber areas.
