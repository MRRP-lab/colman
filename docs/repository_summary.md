## Executive summary

This repository is a ROS 2 Jazzy workspace for a simulated or real UR3e robot with:

- Gazebo simulation
- MoveIt motion planning
- Custom gripper or Robotiq gripper support
- OAK-D camera support
- AprilTag and YOLO perception
- Local Parakeet speech-to-text input (via `onnx-asr`)
- Local Kokoro-82M speech output
- A voice/LLM pipeline: speech, Claude with tool calls, the arm, and spoken replies
- A second, overhead camera and a `world_state` node that tracks which cube is on which

The whole voice-to-arm pipeline is on `main` and `pixi run stack` starts it. The `colman_audio_input` package holds the Parakeet speech node, which mutes the microphone while the robot speaks. The local Kokoro-82M speech-output node lives in `colman_audio_output`. The custom `MoveTo`, `MovePose`, `Pick` and `Place` actions and the `WorldState` message (`colman_interfaces`) and the motion action server (`colman_motion`) are merged, and so is a driver supervisor in `colman_bringup` that recovers the UR arm in place. The `colman_llm` package turns recognised speech into validated arm commands with Claude Sonnet 5.5 (see section 1, "LLM node"). Two OAK-D Pro cameras, one on the wrist and one fixed overhead, feed AprilTag detectors, and `colman_perception`'s `world_state` node fuses them into the `/world_state` topic that the LLM node and the motion server plan from.

The architecture diagram in `docs/architecture/architecture.svg` describes the voice pipeline: speech recognition, a text topic, an AI model with tool calls, a response text topic, and local Kokoro-82M speech synthesis. It also identifies the arm command server as the boundary between AI tool calls and MoveIt. It predates the overhead camera and `world_state`, which it does not show. The design for the LLM node is in [`plan.md`](plan.md).

---

# 1. How the repository functions

## Overall startup flow

The main launch entry point is:

- `colman.launch.py`

It accepts options such as:

```text
sim
moveit
gripper_type
use_camera
depth
yolo
apriltag
```

A normal simulated startup is:

```bash
pixi run sim
```

The real-arm, voice-controlled stack is a separate launch file, `stack.launch.py` (`pixi run stack`). It starts the driver supervisor, the camera publisher, both AprilTag detectors, `world_state`, the motion server, Kokoro, the Parakeet microphone node and `llm_node`. `pixi run stack-dry` is the same without the arm driver and with `dry_run:=true`, for trying the voice and LLM path with no robot connected; it has no joint states, so it does not move anything. See "Operating the real arm".

This launches the following general layers.

## Robot description and controllers

`bringup.launch.py` starts or configures:

- `robot_state_publisher`
- `controller_manager`
- joint-state broadcaster
- arm trajectory controller
- custom or Robotiq gripper controller

The robot description is generated from Xacro:

- simulated robot: `colman_gazebo.xacro`
- real robot: `colman.urdf.xacro`

The controllers are configured in:

- `gripper_controllers.yaml`
- `robotiq_controllers.yaml`

## Gazebo

`sim.launch.py` starts:

- Gazebo
- the robot entity spawner
- ROS-Gazebo bridge
- simulation clock
- simulated camera topics

The bridge configuration is in:

- `bridge_config.yaml`

## MoveIt

`moveit.launch.py` selects either the custom-gripper or Robotiq MoveIt configuration.

The MoveIt configurations live under:

- `src/colman_moveit_config`

The motion demo creates a `MoveItPy` instance and uses it to plan and execute arm trajectories.

## Motion code

The important motion classes are:

- `ArmControl`
- `SceneManager`
- `WorldScene` (keeps one planning-scene box per column of cubes from `/world_state`)
- `TagLookup`
- `VacuumControl`

The current motion demo:

1. Creates a MoveIt planning object.
2. Waits for `/joint_states`.
3. Creates an arm controller.
4. Adds a table collision object.
5. Looks for AprilTag transforms through TF.
6. Moves to an approach position.
7. Lowers the tool.
8. Activates the vacuum through `/io_and_status_controller/set_io`.
9. Retreats.
10. Moves to a destination.
11. Releases the vacuum.

That sequence is still available as a standalone demo (`pixi run demo`), alongside `demo-turns`, `demo-choose` and `demo-circle`.

For other nodes, `motion_server.py` (`pixi run motion`) exposes the same behaviour as ROS interfaces:

- actions `/move_to` (`MoveTo`: `pose_name`), `/pick` (`Pick`: `frame`, optional `place_location`) and `/place` (`Place`: `location`, optional `position`), each returning `success` and `message` and reporting a `stage` as feedback
- services `/stop` and `/resume` (`std_srvs/Trigger`)

`Pick` and `Place` read the fused `tag_N` pose that `world_state` publishes, waiting up to 4 s for it (`FIND_TIMEOUT_S`). `world_state` takes the wrist camera's pose only while the arm is still and the overhead camera's otherwise, so the arm no longer goes to a scan pose first. If the tag is still not there the goal fails with `<tag> not visible`; `HOME_POSE` (`[0, -90°, 0, -90°, 0, 0]`, in `poses.py`) is the seed the IK falls back to when the arm's own posture has no solution. Inverse kinematics is seeded from the arm's current joints, so the arm stays on its current branch. The tool is tried at four quarter turns about the vertical (`TOOL_YAWS_DEG`) so a pick next to a tall stack can find a way in, and a cube is placed square with the one below it. The demos use `locate_tag`.

It accepts one goal at a time and rejects goals while stopped. `pixi run driver` starts the driver supervisor, which restarts the UR driver and recovers the arm without restarting the rest of the stack.

## Perception

The camera node publishes:

```text
/oakd_pro/rgb/image
/oakd_pro/rgb/camera_info
/oakd_pro/left/image
/oakd_pro/right/image
/oakd_pro/left/camera_info_raw
/oakd_pro/right/camera_info_raw
```

The relevant code is in:

- `camera_publisher.py`
- `camera_info_republisher.py`
- `detection.py`

`camera_publisher` has one setting table per camera (`CAMERAS` in `camera_publisher.py`) and three parameters that override it: `fps`, `jpeg_quality` (0 sends raw frames; 1 to 100 has the camera compress each frame as MJPEG at that quality) and `decode_threads` (threads that decode compressed frames). The defaults, from a test with both cameras and both detectors running at once, are:

| Camera | Resolution | Frame rate | Compression | Why |
|---|---|---|---|---|
| wrist | 1080p | 60 (the camera's maximum at 1080p) | none | raw frames are 187 MB/s and none are dropped |
| overhead | 4K | 30 (the camera's maximum at 4K is 42) | MJPEG quality 90, 3 decode threads | raw 4K frames at 30 fps need more than USB 3 carries (27% were dropped); with one decode thread 8 to 24% were dropped |

Both detectors run with `threads: 8` and `decimate: 3.0`. Large images only reach the detectors reliably because `config/fastdds_shared_memory_large_images.xml` is loaded for every pixi task; without it about half of the frames were lost between processes.

AprilTag detection is provided by the external `apriltag_ros` package. The AprilTag launch file remaps the camera input:

- `apriltag.launch.py`: takes `camera:=wrist` (default, node `apriltag`, `apriltag.yaml`, `/oakd_pro` topics) or `camera:=overhead` (node `apriltag_overhead`, `apriltag_overhead.yaml`, `/overhead` topics)
- `overhead.launch.py` (`pixi run overhead`): runs `camera_publisher` with `camera:=overhead` (4K, 30 fps), publishes the `base_link` to `overhead_camera_optical_frame` static transform from `colman_perception/config/overhead_camera_pose_in_base_link.yaml` (the touch-off fit from `camera_pose_calibration/`), and starts the overhead detector. Its tags are named `ovh_tag_N`. At 1.8 m a 39.2 mm tag is only about 63 px wide at 4K (about 30 px at 1080p), so the overhead camera runs at 4K. The detector's depth from tag size reads about 3 cm low, which `overhead_correction.py` corrects with a linear fit; refit it if the tag size, the camera pose or the camera's exposure changes.

The detector config (`colman_perception/config/apriltag.yaml`) registers tag IDs 1 to 8 at 39.2 mm. The motion code finds objects by TF frame names such as:

```text
tag_1   white cube 1
tag_2   white cube 2
tag_3   white cube 3
tag_4   white cube 4
tag_5   blue cube 5
tag_6   blue cube 6
tag_7   blue cube 7
tag_8   blue cube 8
```

That is important for the LLM design: the LLM should map natural language like “the red block” to a known object/frame name. It should not directly invent arbitrary coordinates.

## Voice input and output

The speech input node (`speech_to_text_node.py`) runs NVIDIA Parakeet TDT 0.6B locally through `onnx-asr`, with `silero-vad` for voice activity detection, and needs no API key:

- listens to the microphone
- converts speech to text
- publishes `std_msgs/msg/String`
- mutes the microphone while `tts_speaking` is true, plus a tail, so the robot does not hear itself

By default it publishes on:

```text
/speech_text
```

Parameters include `topic`, `model_name`, `device` (`auto`, `cuda` or `cpu`), `silence_ms`, `max_utterance_s`, `vad_threshold`, `mute_topic` and `mute_tail_ms`. See `src/colman_audio_input/README.md`. `test/wav_file_check.py` runs the node on a WAV file.

It is not included in the main bringup launch file, so it must be started separately:

```bash
ros2 run colman_audio_input speech_to_text
```

The Kokoro output node is `src/colman_audio_output/colman_audio_output/kokoro_tts.py`.

It:

- subscribes to `llm_response` (`std_msgs/msg/String`) by default
- publishes `tts_speaking` (`std_msgs/msg/Bool`)
- converts text to speech locally with Kokoro-82M (Apache 2.0), with no API key or network access after the first weight download
- streams (mpv), plays (ffplay), or saves (`.wav`) the generated audio
- processes synthesis in a worker thread so the ROS subscription callback stays short

Parameters: `input_topic`, `voice` (default `bm_fable`), `lang_code` (default `b`, British; `a` is American), `speed`, `mode` (`stream`, `play` or `save`), `output_dir`, `max_queue`. The `kokoro`, `soundfile` and spaCy model dependencies come from `pixi.toml`; `espeak-ng` is bundled by `espeakng-loader`. Run it with:

```bash
pixi run speak
```

## LLM node

`colman_llm` (`pixi run llm`) subscribes to `/speech_text`, sends each utterance to Claude Sonnet 5.5 (`claude-sonnet-5-5`), and publishes the reply on `llm_response`. It needs `ANTHROPIC_API_KEY`, from the environment or the repository's `.env`, and exits at startup without it. Claude chooses tools, but never talks to ROS. The node validates each call against allow-lists and turns it into an action goal or service call, so it is the safety boundary. Claude never supplies a tag, a coordinate or a topic name.

Tools:

- `stack_cube(color, count)`: build a stack of `count` (2 to 4) `white` or `blue` cubes. It grows the colour's tallest stack with the colour's free cubes in any order; no cube is a base. The node, not Claude, picks which cube moves where.
- `pick_and_place(object, destination)`: move any cube with nothing on top of it onto any other cube, in any colour.
- `go_to_pose(pose)`: `home` or `compact`, through `/move_to`.
- `release_cube`: vacuum off, through `/release`.
- `report_state`: stacks, loose cubes, what the arm holds, and whether it is stopped. It never moves the arm.
- `stop_robot` and `resume_robot`: `/stop` and `/resume`.

Claude speaks for itself. The system prompt tells it to answer with the single word `IGNORE` to anything not about the arm or cubes (the microphone hears a busy room), and to say one short sentence in its own words before `stack_cube`, `pick_and_place` or `go_to_pose`; `ClaudeClient.say` passes any words written before a tool call to the node straight away, so they are spoken while the arm works. The only fixed phrase is "Stopped." on a spoken stop.

Before it picks anything up, the node tells the motion server which cube the picked one will go on (`place_location` in the `Pick` goal). The server finds that cube's tag and solves the drop and approach poses it will use for the place, with the cube modelled on the tip and the destination column left out of the scene, and fails the pick with "No safe arm pose to place on `<tag>`" before moving if there is none. The arm is never left holding a cube it cannot put down. See [`arm_reach/README.md`](arm_reach/README.md) for where the arm can reach on the table.

Saying "stop", "halt", "freeze", "cancel", "abort", "hold still", "stay still", "don't move" or "that's enough" stops the arm immediately, before anything is queued: the node calls `/stop`, cancels the active goal and drops the queue. Parameters: `model`, `pick_timeout_s`, `place_timeout_s`, `move_timeout_s` and `dry_run`. With `dry_run:=true` a fake executor returns canned results, so it runs without an arm. Which cube is on which comes from the `/world_state` topic (`colman_perception/world_state.py`), built from the overhead camera, so moving a cube by hand is seen. The node refuses to move without a fresh `/world_state`. The cubes are exactly 50 mm and the table surface is at `base_link` z = -0.005, so a cube at level n has its top at exactly -0.005 + 0.05 n. These numbers (`CUBE_SIZE_M`, `TAG_SIZE_M`, `TABLE_TOP_Z`, `cube_top_z(level)`) are defined once, in `colman_description/workcell.py`, and every package imports them: `world_state` publishes `tag_N` at the exact height of the cube's level (the camera's own height only picks the level), the motion server's drop height, held-cube box and table box use them, the Gazebo cube models are 50 mm, and tests fail if the 39.2 mm tag size is used as a cube size. The motion server also reads `/world_state` and keeps one box per column of cubes in MoveIt's planning scene (`colman_motion/world_scene.py`), so the arm plans around stacks it is not working on; the column being picked from or placed on is left out while that move runs, and a stale world state keeps the last known boxes. If the overhead camera stops (no overhead detections for 5 s), `/world_state` is marked stale, `world_state` stops publishing `tag_N` for remembered cubes so the motion server cannot act on a guess, and after an outage of 10 s or more the order inside every stack is marked uncertain until the cubes are seen again. Details are in [`plan.md`](plan.md).

## Operating the real arm

`pixi run stack` starts everything for the voice-controlled arm (see "Overall startup flow"). It can also be run as separate processes, each its own pixi task, started in this order: `driver`, `motion`, `detect` (camera, `world_state` and the wrist AprilTag detector), `overhead`, then optionally `image-view`. Only one driver may hold the robot, because a second one fails with `speed_slider_mask is currently controlled by another RTDE client`. For a manual pick, send `ros2 action send_goal --feedback /pick colman_interfaces/action/Pick "{frame: tag_1}"`, then `/place` or `/release`. A pick that succeeds leaves the cube held, and the next pick is refused until it is placed or released.

---

# 2. ROS communication summary

| Component | Interface | Purpose |
|---|---|---|
| Speech input (Parakeet) | Publishes `std_msgs/msg/String` on `/speech_text` | Human speech converted to text |
| LLM node | Subscribes `/speech_text`, `/world_state`; publishes `llm_response` | Claude with validated tool calls; clients for `/pick`, `/place`, `/move_to`, `/stop`, `/resume`, `/release` |
| `world_state` | Subscribes to both detectors' tags and `/joint_states`; publishes `tag_N` TF frames and `/world_state` (`colman_interfaces/msg/WorldState`) | Which cube is on which, with the arm's hiding of cubes accounted for |
| Kokoro speech output | Subscribes to `std_msgs/msg/String` on `llm_response` | Text converted to spoken audio |
| Kokoro speech output | Publishes `std_msgs/msg/Bool` on `tts_speaking` | Indicates when speech playback is active |
| Camera publisher | Publishes `sensor_msgs/msg/Image` | RGB/stereo camera images |
| Camera publisher | Publishes `sensor_msgs/msg/CameraInfo` | The camera's factory lens intrinsics (focal length, principal point, distortion) |
| YOLO node | Subscribes to `/oakd_pro/rgb/image` | Runs object detection |
| YOLO node | Publishes `/yolo/annotated` | Annotated image visualization |
| AprilTag node | Subscribes to camera image/camera info | Detects AprilTags |
| AprilTag/TF | Publishes TF frames | Makes tag locations available to motion code |
| Robot state publisher | Publishes TF and robot state | Robot kinematic transforms |
| Joint state broadcaster | Publishes `/joint_states` | Current robot joint positions |
| MoveIt | Uses planning/action interfaces | Plans and executes motion |
| Vacuum control | Calls `/io_and_status_controller/set_io`; provides services `/grasp`, `/release` | Turns vacuum on/off |
| Motion server | Provides actions `/move_to`, `/move_pose`, `/pick`, `/place` | Arm commands for other nodes |
| Motion server | Provides services `/stop`, `/resume` | Halts the arm and re-enables goals |

The stop interface is a service, not an action (`pixi run stop` and `pixi run resume` wrap these):

```bash
ros2 service call /stop std_srvs/srv/Trigger "{}"
ros2 service call /resume std_srvs/srv/Trigger "{}"
```

---

---

# 3. Implementation order

## Phase 1: make the motion interface callable

`MoveTo`, `Pick` and `Place` and the motion server are done (PR #14).

1. Test them manually with `ros2 action send_goal`. (Done)
2. The LLM `pick_and_place` tool calls `/pick` then `/place`. No `PickAndPlace.action` is added (see [`plan.md`](plan.md)). (Done)

## Phase 2: add text-only LLM interaction (done)

Before involving speech, test:

```bash
ros2 topic pub --once /speech_text std_msgs/msg/String \
  "{data: 'pick up the red block and put it in the left tray'}"
```

The LLM node should publish:

```text
/llm_response
```

At this stage, inspect the response with:

```bash
ros2 topic echo /llm_response
```

## Phase 3: package and connect audio output (done)

Add the packaged `colman_audio_output` node (`pixi run speak`) to the launch flow, and verify:

```text
speech_text -> LLM -> llm_response -> Kokoro -> speaker
```

## Phase 4: connect microphone input (done)

Finally launch speech recognition and verify the complete loop. Parakeet (through `onnx-asr`, with `silero-vad` for voice activity detection) replaced the Azure node, and mutes the microphone while `tts_speaking` is true so the robot does not hear itself. `pixi run stack` now runs the whole loop.

This ordering makes debugging much easier because each layer can be tested independently.

---

---

# 4. Remaining work

The LLM package, the Parakeet node, the overhead camera, `world_state` and the camera-pose calibration are all merged. Still open:

1. Choose a license. The repository has no LICENSE file, and every `package.xml` and `setup.py` still says `TODO: License declaration`.
2. Redraw the architecture diagram: it does not show the overhead camera, `world_state`, or the calibration tools.
3. The overhead camera's exposure clips the white tag cells, and the tag depth reads about 3 cm low (corrected, not fixed at the source). Lower the exposure and refit `overhead_correction.py`.
4. The calibrations are only checked at table height. Heights above the table (stacks) have no independent ground truth beyond the correction's fit.
5. Run the hardware-in-the-loop checklist (`docs/hil_checklist.md`) on the real arm and keep the results.
6. Decide which stale remote branches to delete. Several contain unmerged work, so do not delete them blindly.
7. CI skips the ament flake8 and pep257 tests, and lints only `src/` while `pixi run lint` also covers `scripts/`.
1. Choose a license. The repository has no LICENSE file, and several `package.xml` and `setup.py` files still say `TODO: License declaration`.
2. Add the `colman_llm` package, following [`plan.md`](plan.md).
3. Merge the `NVIDIA-Parakeet` branch, which replaces the Azure speech input with a local Parakeet node and mutes the microphone while the robot speaks.
4. Decide which stale remote branches to delete. Several contain unmerged work, so do not delete them blindly.
5. Keep the architecture diagram synchronized with the implementation.

### The LLM speaks up when the table changes by itself

Every request to Claude carries the current table (`/world_state`, plus what the arm
holds) as a system block, rebuilt each time and never kept in the history. `world_watch.py`
also watches `/world_state` between requests and publishes a sentence on `llm_response`
when a change lasts 3 s: a cube lost or found, or one moved by someone. It stays quiet
while the arm is working, holds a cube or is being spoken to, waits for the table to be
still for 3 s before it takes a baseline, spaces announcements 10 s apart, and says once
when the camera is lost and once when it is back. What it said joins the conversation
history so a follow-up like "put it back" makes sense. The node parameter
`announce_changes` (default true) turns it off.
