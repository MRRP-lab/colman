## Executive summary

This repository is a ROS 2 Jazzy workspace for a simulated or real UR3e robot with:

- Gazebo simulation
- MoveIt motion planning
- Custom gripper or Robotiq gripper support
- OAK-D camera support
- AprilTag and YOLO perception
- Local Parakeet speech-to-text input (via `onnx-asr`)
- A partially implemented voice/LLM architecture

The workspace builds successfully, but the LLM/voice-command portion is incomplete. The local Kokoro-82M speech-output node lives in the `colman_audio_output` package. The custom `MoveTo`, `Pick` and `Place` actions (`colman_interfaces`) and the motion action server (`colman_motion`) are merged (PR #14). A driver supervisor in `colman_bringup` recovers the UR arm in place (PR #13). There is still no `colman_llm` package.

Dependencies for local Parakeet speech recognition (`onnx-asr`, `onnxruntime-gpu`, `silero-vad`, `sounddevice`, `huggingface_hub`, plus `portaudio`) are in `pixi.toml`, but no node uses them yet. `speech_to_text_node.py` still uses Azure and is to be replaced by a Parakeet node.

The architecture diagram in `docs/architecture/architecture.svg` describes the planned voice pipeline: speech recognition, a text topic, an AI model with tool calls, a response text topic, and local Kokoro-82M speech synthesis. It also identifies the arm command server as the boundary between AI tool calls and MoveIt. The design for the LLM node is in [`plan.md`](plan.md).

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

AprilTag detection is provided by the external `apriltag_ros` package. The AprilTag launch file remaps the camera input:

- `apriltag.launch.py`

The motion code then finds objects by TF frame names such as:

```text
tag_1
tag_3
tag_5
tag_6
```

That is important for the LLM design: the LLM should map natural language like “the red block” to a known object/frame name. It should not directly invent arbitrary coordinates.

## Voice input and output

The speech input node (`speech_to_text_node.py`, currently Azure-based and to be replaced by a local Parakeet node):

- listens to the microphone
- converts speech to text
- publishes `std_msgs/msg/String`

By default it publishes on:

```text
/speech_text
```

The relevant input code is:

- `speech_to_text_node.py`

It is not currently included in the main bringup launch file, so it must be started separately:

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

Parameters: `input_topic`, `voice` (default `af_heart`), `lang_code`, `speed`, `mode` (`stream`, `play` or `save`), `output_dir`, `max_queue`. The `kokoro`, `soundfile` and spaCy model dependencies come from `pixi.toml`; `espeak-ng` is bundled by `espeakng-loader`. Run it with:

```bash
pixi run speak
```

---

# 2. ROS communication summary

| Component | Interface | Purpose |
|---|---|---|
| Speech input (Parakeet; current node is Azure) | Publishes `std_msgs/msg/String` on `/speech_text` | Human speech converted to text |
| Kokoro speech output | Subscribes to `std_msgs/msg/String` on `llm_response` | Text converted to spoken audio |
| Kokoro speech output | Publishes `std_msgs/msg/Bool` on `tts_speaking` | Indicates when speech playback is active |
| Camera publisher | Publishes `sensor_msgs/msg/Image` | RGB/stereo camera images |
| Camera publisher | Publishes `sensor_msgs/msg/CameraInfo` | Camera calibration |
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
3. Replace the Azure speech input with a local Parakeet node (the dependencies are already in `pixi.toml`).
4. Decide which stale remote branches to delete. Several contain unmerged work, so do not delete them blindly.
5. Keep the architecture diagram synchronized with the implementation.
