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

The workspace builds and the whole voice pipeline exists. The `colman_audio_input` package holds the Parakeet speech node, which mutes the microphone while the robot speaks. The local Kokoro-82M speech-output node lives in `colman_audio_output`. The custom `MoveTo`, `Pick` and `Place` actions (`colman_interfaces`) and the motion action server (`colman_motion`) are merged (PR #14), and so is a driver supervisor in `colman_bringup` that recovers the UR arm in place (PR #13). The `colman_llm` package turns recognised speech into validated arm commands with Claude Sonnet 5.5 (see section 1, "LLM node"). The `llm` branch carries `colman_llm`, which is not yet merged into `main`; `main` has only its scaffold.

The architecture diagram in `docs/architecture/architecture.svg` describes the voice pipeline: speech recognition, a text topic, an AI model with tool calls, a response text topic, and local Kokoro-82M speech synthesis. It also identifies the arm command server as the boundary between AI tool calls and MoveIt. The design for the LLM node is in [`plan.md`](plan.md).

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

- actions `/move_to` (`MoveTo`: `pose_name`), `/pick` (`Pick`: `frame`) and `/place` (`Place`: `location`, optional `position`), each returning `success` and `message` and reporting a `stage` as feedback
- services `/stop` and `/resume` (`std_srvs/Trigger`)

`Pick` and `Place` find their tag without a scan pose (`scan.py`). The wrist camera gets one look at once, from wherever the arm is. If it does not see the tag, the overhead camera is used (`ovh_tag_N`), and its pose is picked and placed from directly. If neither sees it the arm may be blocking the overhead view, so it moves to `HOME_POSE` (`[0, -90°, 0, -90°, 0, 0]`, in `poses.py`) and the overhead camera is tried again for up to 4 s; if it is still not seen the goal fails with `<tag> not visible`. Inverse kinematics is seeded from the arm's current joints, so the arm stays on its current branch. The demos use `locate_tag`, which does the same.

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
- `overhead.launch.py` (`pixi run overhead`): runs `camera_publisher` with `camera:=overhead` (4K, 30 fps), publishes the `base_link` to `overhead_camera_optical_frame` static transform from `colman_perception/config/overhead_camera_pose_in_base_link.yaml` (the merged calibration), and starts the overhead detector. Its tags are named `ovh_tag_N`. At 1.8 m a 39.2 mm tag is about 30 px wide at 1080p; all eight tags are detected.

The detector config (`colman_perception/config/apriltag.yaml`) registers tag IDs 1 to 8 at 39.2 mm. The motion code finds objects by TF frame names such as:

```text
tag_1   white cube 1 (base of the white stack)
tag_2   white cube 2
tag_3   white cube 3
tag_4   white cube 4
tag_5   blue cube 1 (base of the blue stack)
tag_6   blue cube 2
tag_7   blue cube 3
tag_8   blue cube 4
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

Parameters: `input_topic`, `voice` (default `am_echo`), `lang_code`, `speed`, `mode` (`stream`, `play` or `save`), `output_dir`, `max_queue`. The `kokoro`, `soundfile` and spaCy model dependencies come from `pixi.toml`; `espeak-ng` is bundled by `espeakng-loader`. Run it with:

```bash
pixi run speak
```

## LLM node

`colman_llm` (`pixi run llm`) subscribes to `/speech_text`, sends each utterance to Claude Sonnet 5.5 (`claude-sonnet-5-5`), and publishes the reply on `llm_response`. It needs `ANTHROPIC_API_KEY`, from the environment or the repository's `.env`, and exits at startup without it. Claude chooses tools, but never talks to ROS. The node validates each call against allow-lists and turns it into an action goal or service call, so it is the safety boundary. Claude never supplies a tag, a coordinate or a topic name.

Tools:

- `stack_cube(color, count)`: build a stack of `white` or `blue` cubes, 1 to 3 on top of that colour's base cube. The node, not Claude, picks which cube moves where.
- `pick_and_place(object, destination)`: move one loose cube onto any cube. Bases never move.
- `go_to_pose(pose)`: `upright` or `compact`, through `/move_to`.
- `release_cube`: vacuum off, through `/release`.
- `report_state`: stacks, loose cubes, what the arm holds, and whether it is stopped. It never moves the arm.
- `stop_robot` and `resume_robot`: `/stop` and `/resume`.

Saying "stop", "halt" or "freeze" stops the arm immediately, before anything is queued: the node calls `/stop`, cancels the active goal and drops the queue. Parameters: `model`, `pick_timeout_s`, `place_timeout_s`, `move_timeout_s` and `dry_run`. With `dry_run:=true` a fake executor returns canned results, so it runs without an arm. Which cube is on which comes from the `/world_state` topic (`colman_perception/world_state.py`), built from the overhead camera, so moving a cube by hand is seen. The node refuses to move without a fresh `/world_state`. The cubes are exactly 50 mm and the table surface is at `base_link` z = -0.005, so a cube at level n has its top at exactly -0.005 + 0.05 n. These numbers (`CUBE_SIZE_M`, `TAG_SIZE_M`, `TABLE_TOP_Z`, `cube_top_z(level)`) are defined once, in `colman_description/workcell.py`, and every package imports them: `world_state` publishes `tag_N` at the exact height of the cube's level (the camera's own height only picks the level), the motion server's drop height, held-cube box and table box use them, the Gazebo cube models are 50 mm, and tests fail if the 39.2 mm tag size is used as a cube size. The motion server also reads `/world_state` and keeps one box per column of cubes in MoveIt's planning scene (`colman_motion/world_scene.py`), so the arm plans around stacks it is not working on; the column being picked from or placed on is left out while that move runs, and a stale world state keeps the last known boxes. If the overhead camera stops (no overhead detections for 5 s), `/world_state` is marked stale, `world_state` stops publishing `tag_N` for remembered cubes so the motion server cannot act on a guess, and after an outage of 10 s or more the order inside every stack is marked uncertain until the cubes are seen again. Details are in [`plan.md`](plan.md).

## Operating the real arm

The real-arm stack is several separate processes, each its own pixi task, started in this order: `driver`, `motion`, `detect` (camera and AprilTag), then optionally `image-view`. Only one driver may hold the robot, because a second one fails with `speed_slider_mask is currently controlled by another RTDE client`. For a manual pick, send `ros2 action send_goal --feedback /pick colman_interfaces/action/Pick "{frame: tag_1}"`, then `/place` or `/release`. A pick that succeeds leaves the cube held, and the next pick is refused until it is placed or released.

---

# 2. ROS communication summary

| Component | Interface | Purpose |
|---|---|---|
| Speech input (Parakeet; current node is Azure) | Publishes `std_msgs/msg/String` on `/speech_text` | Human speech converted to text |
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
| Vacuum control | Calls `/io_and_status_controller/set_io` | Turns vacuum on/off |
| Motion server | Provides actions `/move_to`, `/pick`, `/place` | Arm commands for other nodes |
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

1. Test them manually with `ros2 action send_goal`.
2. The LLM `pick_and_place` tool calls `/pick` then `/place`. No `PickAndPlace.action` is added (see [`plan.md`](plan.md)).

## Phase 2: add text-only LLM interaction

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

## Phase 3: package and connect audio output

Add the packaged `colman_audio_output` node (`pixi run speak`) to the launch flow, and verify:

```text
speech_text -> LLM -> llm_response -> Kokoro -> speaker
```

## Phase 4: connect microphone input

Finally launch speech recognition and verify the complete loop. Parakeet (through `onnx-asr`, with `silero-vad` for voice activity detection) replaces the Azure node here. Mute the microphone while `tts_speaking` is true so the robot does not hear itself.

This ordering makes debugging much easier because each layer can be tested independently.

---

---

# 4. Remaining work

The repository cleanup from the original review is complete (PRs #9 and #12). Still open:

1. Choose a license. The repository has no LICENSE file, and several `package.xml` and `setup.py` files still say `TODO: License declaration`.
2. Add the `colman_llm` package, following [`plan.md`](plan.md).
3. Merge the `NVIDIA-Parakeet` branch, which replaces the Azure speech input with a local Parakeet node and mutes the microphone while the robot speaks.
4. Decide which stale remote branches to delete. Several contain unmerged work, so do not delete them blindly.
5. Keep the architecture diagram synchronized with the implementation.
