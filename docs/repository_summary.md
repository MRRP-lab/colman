## Executive summary

This repository is a ROS 2 Jazzy workspace for a simulated or real UR3e robot with:

- Gazebo simulation
- MoveIt motion planning
- Custom gripper or Robotiq gripper support
- OAK-D camera support
- AprilTag and YOLO perception
- Azure speech-to-text input
- A partially implemented voice/LLM architecture

The workspace builds successfully, but the LLM/voice-command portion is incomplete. The ElevenLabs speech-output node is being packaged as `colman_audio_output` in open PR #8. The custom `MoveTo`, `Pick` and `Place` actions and a motion action server are in open PR #14. There is still no `colman_llm` package on `main`.

The architecture diagram in `docs/architecture/architecture.svg` describes the planned voice pipeline: speech recognition, a text topic, an AI model with tool calls, a response text topic, and ElevenLabs speech synthesis. It also identifies the arm command server as the boundary between AI tool calls and MoveIt. The design for the LLM node is in [`plan.md`](plan.md).

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

The important limitation is that this behavior is hard-coded into a process. Other ROS nodes cannot request “pick up tag 3 and put it at tag 1” through a clean interface on `main`; PR #14 adds `MoveTo`, `Pick` and `Place` actions for this.

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

The Azure node:

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

The ElevenLabs output node is:

- `src/colman_audio_output/eleven_labs.py` on `main`
- `colman_audio_output/eleven_labs.py` inside an installable package in open PR #8

It:

- subscribes to `llm_response` (`std_msgs/msg/String`) by default
- publishes `tts_speaking` (`std_msgs/msg/Bool`)
- converts text through ElevenLabs
- plays, streams, or saves the generated audio
- processes synthesis in a worker thread so the ROS subscription callback stays short

It needs `ELEVENLABS_API_KEY` and the ElevenLabs Python SDK. Once PR #8 is merged it runs with:

```bash
ros2 run colman_audio_output eleven_labs
```

---

# 2. ROS communication summary

| Component | Interface | Purpose |
|---|---|---|
| Azure speech input | Publishes `std_msgs/msg/String` on `/speech_text` | Human speech converted to text |
| ElevenLabs speech output | Subscribes to `std_msgs/msg/String` on `llm_response` | Text converted to spoken audio |
| ElevenLabs speech output | Publishes `std_msgs/msg/Bool` on `tts_speaking` | Indicates when speech playback is active |
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
| Arm control | Provides `/stop` | Stops current MoveIt execution |

The stop interface is a service, not an action:

```bash
ros2 service call /stop std_srvs/srv/Trigger "{}"
```

---

---

# 3. Suggested implementation order

## Phase 1: make the motion interface callable

PR #14 covers much of this phase.

1. Merge `colman_interfaces` and the motion action server (PR #14).
2. Add a single `PickAndPlace.action` (see [`plan.md`](plan.md)).
3. Test it manually with `ros2 action send_goal`.

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

Merge the packaged `colman_audio_output` node (PR #8), add it to the launch flow, and verify:

```text
speech_text -> LLM -> llm_response -> ElevenLabs -> speaker
```

## Phase 4: connect microphone input

Finally launch Azure speech recognition and verify the complete loop.

This ordering makes debugging much easier because each layer can be tested independently.

---

---

# 4. Remaining work

The repository cleanup from the original review is complete (PRs #9 and #12). Still open:

1. Merge the audio output package (PR #8) and the action server (PR #14).
2. Choose a license. The repository has no LICENSE file, and several `package.xml` and `setup.py` files still say `TODO: License declaration`.
3. Add a `PickAndPlace` action and the `colman_llm` package, following [`plan.md`](plan.md).
4. Decide which stale remote branches to delete. Several contain unmerged work, so do not delete them blindly.
5. Keep the architecture diagram synchronized with the implementation.
