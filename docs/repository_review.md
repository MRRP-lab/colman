## Executive summary

This repository is a ROS 2 Jazzy workspace for a simulated or real UR3e robot with:

- Gazebo simulation
- MoveIt motion planning
- Custom gripper or Robotiq gripper support
- OAK-D camera support
- AprilTag and YOLO perception
- Azure speech-to-text input
- A partially implemented voice/LLM architecture

The workspace currently builds successfully, but the LLM/voice-command portion is incomplete. A ROS-compatible ElevenLabs speech-output script now exists at `src/colman_audio_output/eleven_labs.py`, but it is not yet an installable ROS package. There is still no `colman_llm` package and no custom `pick_and_place` ROS action in `main`.

The current architecture diagram in `docs/architecture/architecture.svg` describes a planned voice pipeline: speech recognition, a text topic, an AI model with tool calls, a response text topic, and ElevenLabs speech synthesis. It also identifies the arm command server as the missing boundary between AI tool calls and MoveIt. The historical branch [`origin/eyeinhand`](https://github.com/MRRP-lab/colman/tree/eyeinhand) contains a useful but unmerged custom action-server implementation with `MoveTo`, `Pick`, and `Place` actions. That is probably the best starting point for the command-server portion.

---

# 1. Repository cleanup and fixes

## High-priority issues

### 1.1 LLM implementation is missing; audio output is not packaged

The architecture diagram now shows the intended voice path, but only part of that path is implemented:

- `src/colman_audio_input/colman_audio_input/speech_to_text_node.py` is an installable Azure speech-input ROS node.
- `src/colman_audio_output/eleven_labs.py` is an ElevenLabs ROS node implementation, but it lives outside an `ament_python` package and is not exposed through `ros2 run`.
- `docs/architecture/architecture.svg` and `docs/architecture/architecture.png` document the planned pipeline.

The following are currently absent:

- `src/colman_llm`
- Claude API integration
- An AI node that turns text into tool calls
- An LLM launch file
- A custom high-level manipulation action

The audio-output script currently subscribes to `llm_response` by default, publishes `tts_speaking`, and supports ElevenLabs `stream`, `play`, and `save` modes. It requires `ELEVENLABS_API_KEY` and an installed ElevenLabs Python SDK. It should either be moved into a real `colman_audio_output` package or clearly documented as a standalone script.

### 1.2 No custom manipulation actions exist on the current branch

The current branch does not contain:

- `colman_interfaces`
- `.action` definitions
- `ActionServer`
- `ActionClient`
- `pick_and_place` action server

The current pick-and-place code is a standalone executable:

- `pick_and_place.py`

It runs a hard-coded demo and is not callable by another ROS node with a target and destination.

### 1.3 Duplicate `colman_bringup` package tree

There are two copies of the bringup package:

- `src/colman_bringup`
- `src/colman_gazebo/colman_bringup`

They are duplicates. The nested copy should be removed after confirming nobody depends on it. Keeping both creates confusion and risks package-discovery problems.

The workspace currently discovers only one `colman_bringup`, which makes the duplicate especially misleading rather than useful.

### 1.4 Motion package dependency declarations are incomplete

`colman_motion/package.xml` declares only:

- `rclpy`
- `sensor_msgs`

However, the motion package directly imports or uses:

- MoveIt Python APIs
- `geometry_msgs`
- `moveit_msgs`
- `shape_msgs`
- `tf2_ros`
- `ur_msgs`
- `std_srvs`

The build succeeds because the Pixi environment already provides the dependencies transitively, but the package is not self-describing. These dependencies should be declared explicitly.

### 1.5 `colman_gazebo` does not install its models

`src/colman_gazebo/setup.py` installs worlds, configs, and URDF files, but not the Gazebo models under:

```text
src/colman_gazebo/colman_gazebo/models/
```

The current Pixi environment works around this by setting `GZ_SIM_RESOURCE_PATH` to the source directory in `pixi.toml`.

That means the package is less portable than it should be. Either:

1. install the models through `setup.py`, or
2. explicitly document that the workspace must always be run from source with the Pixi environment activated.

Installing the models is preferable.

### 1.6 Azure environment variables have misleading names

The speech-input node uses:

```text
AZURE_TTS_KEY
AZURE_TTS_REGION
```

even though this is speech recognition, not text-to-speech.

These should probably become:

```text
AZURE_SPEECH_KEY
AZURE_SPEECH_REGION
```

A compatibility fallback could temporarily support the old names.

### 1.7 Current lint failures

`pixi run ruff check src` fails with 12 issues, mainly:

- import ordering
- unused `time` imports
- lines longer than 88 characters

Affected files include:

- `speech_to_text_node.py`
- `pick_and_place.py`
- `demo_choose.py`
- `demo_turns.py`

These are straightforward cleanup items.

### 1.8 Pixi metadata warnings

Pixi currently warns that:

- `[system-requirements]` is deprecated
- `pixi.lock` uses an older lockfile format

This does not currently prevent building, but it should eventually be updated in `pixi.toml` and `pixi.lock`.

## Lower-priority cleanup

These tracked files may be worth archiving or removing if they are no longer part of the project:

- `docs/garbageIntroPresentation.pptx`
- `docs/questions_old.md`
- `calib_samples.npz`

`calib_samples.npz` looks like generated calibration data. It should either be:

- documented as an intentional calibration fixture,
- moved under a clearly named data directory, or
- removed from git and regenerated when needed.

Many package metadata files still contain:

```text
TODO: Package description
TODO: License declaration
```

Those should be cleaned up before the repository is considered production-ready.

## Branch cleanup

The local branches are:

```text
main
llm
```

Both currently point to the same commit:

```text
1b249cf Save local changes before revert
```

Therefore, the local `llm` branch has no unique work at this moment. If it is not reserved for upcoming LLM development, it can be deleted and development can continue from `main`.

The remote branches should not be deleted blindly. Several contain unmerged work, including:

- `origin/eyeinhand`
- `origin/PicknPlace`
- `origin/final-project`
- `origin/real-robot`
- `origin/yolo-demo`

The most relevant one for the LLM work is `origin/eyeinhand`, because it contains the custom motion action-server design.

---

# 2. How the repository functions

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

The important limitation is that this behavior is hard-coded into a process. Other ROS nodes cannot request “pick up tag 3 and put it at tag 1” through a clean interface yet.

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

The new ElevenLabs output implementation is:

- `src/colman_audio_output/eleven_labs.py`

It currently:

- subscribes to `llm_response` (`std_msgs/msg/String`) by default
- publishes `tts_speaking` (`std_msgs/msg/Bool`)
- converts text through ElevenLabs
- plays, streams, or saves the generated audio
- processes synthesis in a worker thread so the ROS subscription callback stays short

It is not currently a ROS package, so this does not work yet:

```bash
ros2 run colman_audio_output eleven_labs
```

The simplest packaging fix is to add `package.xml`, `setup.py`, `setup.cfg`, a resource marker, and an entry point such as `eleven_labs = colman_audio_output.eleven_labs:main`. The package should also declare `std_msgs` and document the external ElevenLabs and playback dependencies.

---

# 3. ROS communication summary

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

# 4. What ROS actions are currently available?

## Custom actions: none on `main`

There are currently no repository-defined actions on the active branch.

In particular, this command would not currently show a Colman-specific manipulation action:

```bash
ros2 action list
```

The current pick-and-place executable is not an action server.

## Controller actions available at runtime

Depending on which controllers are running, the controller manager will generally expose actions similar to:

```text
/joint_trajectory_controller/follow_joint_trajectory
/gripper_controller/gripper_cmd
```

or, for Robotiq:

```text
/robotiq_gripper_controller/gripper_cmd
```

The exact names should always be verified at runtime:

```bash
ros2 action list -t
```

### Arm trajectory action

The arm controller normally uses:

```text
control_msgs/action/FollowJointTrajectory
```

Its goal contains:

- `trajectory.joint_names`
- `trajectory.points`
- optional path tolerances
- optional goal tolerances
- optional goal time tolerance

A simplified command looks like:

```bash
ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [...]}}"
```

This is a low-level interface. The LLM should not call it directly. MoveIt or a Colman motion server should handle trajectory generation.

### Gripper action

The gripper controller normally uses:

```text
control_msgs/action/GripperCommand
```

A simplified command is:

```bash
ros2 action send_goal \
  /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 20.0}}"
```

Again, this should be hidden behind a higher-level Colman action.

## Historical custom actions

The historical `origin/eyeinhand` branch contains these definitions.

### `MoveTo.action`

```text
# Goal
string pose_name
---
# Result
bool success
---
# Feedback
```

Example:

```bash
ros2 action send_goal \
  /move_to \
  colman_interfaces/action/MoveTo \
  "{pose_name: Look3}"
```

### `Pick.action`

```text
# Goal
string frame
---
# Result
bool success
string message
---
# Feedback
string stage
```

Example:

```bash
ros2 action send_goal \
  /pick \
  colman_interfaces/action/Pick \
  "{frame: tag_3}"
```

Feedback stages include values such as:

```text
looking
detecting
approaching
lowering
grasping
retreating
```

### `Place.action`

```text
# Goal
string location
geometry_msgs/Point position
---
# Result
bool success
string message
---
# Feedback
string stage
```

Example using a named destination:

```bash
ros2 action send_goal \
  /place \
  colman_interfaces/action/Place \
  "{location: tag_1, position: {x: 0.0, y: 0.0, z: 0.0}}"
```

The historical motion server starts:

```text
/move_to
/pick
/place
```

That design is much closer to what the LLM needs than the current standalone demo.

---

# 5. Recommended initial `colman_llm` design

I would avoid building a general-purpose robot agent at first. The simplest reliable architecture is:

```text
microphone
   |
   v
colman_audio_input
   |
   | /speech_text
   v
colman_llm
   |
   | LLM tool-use request
   |
   +--> arm command/action client
   |
   +--> other future action clients
   |
   | llm_response
   v
colman_audio_output / ElevenLabs
   |
   v
speaker
```

## Recommended package layout

Create:

```text
src/colman_llm/
  colman_llm/
    __init__.py
    llm_node.py
  resource/colman_llm
  package.xml
  setup.py
```

Initially, keep the implementation in one node.

The node should:

1. Subscribe to `/speech_text`.
2. Send the text to the selected model.
3. Give the model a small, explicit list of tools.
4. If the model requests a tool, call the corresponding ROS action or service.
5. Send the action result back to the model.
6. Publish the final response on `llm_response`, matching the current ElevenLabs node's default subscription.

The updated architecture diagram labels the model as OpenAI `gpt-4o-mini`, while the original project goal described Claude. This is an implementation choice, not a ROS constraint. Use one provider initially and keep provider-specific API calls behind a small adapter; do not implement both providers until the end-to-end ROS flow works.

## Claude tool definitions

Initially expose only one tool:

```json
{
  "name": "pick_and_place",
  "description": "Pick up a known object and place it at a known destination.",
  "input_schema": {
    "type": "object",
    "properties": {
      "object": {
        "type": "string",
        "description": "Known object name or AprilTag frame"
      },
      "destination": {
        "type": "string",
        "description": "Known destination name or AprilTag frame"
      }
    },
    "required": ["object", "destination"]
  }
}
```

The node should validate the result against an allow-list, for example:

```python
OBJECT_FRAMES = {
    "red block": "tag_3",
    "blue block": "tag_5",
}

DESTINATION_FRAMES = {
    "left tray": "tag_1",
    "right tray": "tag_6",
}
```

Do not let Claude directly supply arbitrary:

- joint positions
- Cartesian coordinates
- ROS topic names
- service names
- action names
- Python code

Claude should choose from a small set of safe tools, and the ROS node should translate those choices into known action goals.

## One action versus separate `Pick` and `Place`

For the first implementation, I recommend one high-level action:

```text
PickAndPlace.action
```

For example:

```text
# Goal
string object_frame
string destination_frame
---
# Result
bool success
string message
---
# Feedback
string stage
```

The action server would internally:

1. call the existing pick logic
2. call the existing place logic
3. report progress through `stage`

This is easier for Claude to use than requiring it to reason about two dependent actions.

The historical `Pick` and `Place` actions can still be reused internally, but the LLM should initially see one tool.

A future version could expose separate tools:

- `move_to_named_pose`
- `pick_object`
- `place_object`
- `stop_robot`

## Important implementation detail: do not block ROS callbacks

The Claude API and robot motion actions can take seconds or minutes. The speech subscription callback should not perform the entire interaction synchronously.

Use:

- a worker thread, or
- an asyncio task with an appropriate executor arrangement

The basic flow should be:

```text
ROS subscription callback
    -> queue user text
    -> worker calls Claude
    -> worker waits for ROS action result
    -> worker publishes final response
```

This keeps the ROS executor responsive.

## Audio output

The repository now has the core of a small `colman_audio_output` node in `src/colman_audio_output/eleven_labs.py`. After packaging it, the node should:

- subscribe to `llm_response`
- converts text to speech
- plays it
- publish `tts_speaking` while playback is active

The existing script already supports stream, play, and save modes. For an initial version, use one provider and one mode. Do not add interruption, caching, and multi-provider fallback until the basic loop works. The `tts_speaking` topic can later coordinate microphone muting, but it should not be required for the first closed-loop test.

The speech loop would be:

```text
User: "Pick up the block and put it in the tray."
    |
    v
/speech_text
    |
    v
The model decides to call an arm tool
    |
    v
ROS action executes
    |
    v
The model receives success/failure
    |
    v
/llm_response: "I picked up the block and placed it in the tray."
    |
    v
ElevenLabs speaks the response
```

## Model API credentials

Use the environment variable for the selected provider. For Claude:

```bash
export ANTHROPIC_API_KEY="..."
```

For the OpenAI model shown in the current architecture diagram:

```bash
export OPENAI_API_KEY="..."
```

Do not place the key in:

- `pixi.toml`
- launch files
- source code
- YAML files
- git history

The selected model SDK should be added through the project’s dependency management rather than installed ad hoc.

---

# 6. Suggested implementation order

## Phase 1: make the motion interface callable

1. Remove the duplicate nested `colman_bringup`.
2. Restore or adapt the historical `colman_interfaces` package.
3. Add a single `PickAndPlace.action`.
4. Refactor the hard-coded pick-and-place logic into an action server.
5. Test it manually with `ros2 action send_goal`.

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

Package `src/colman_audio_output/eleven_labs.py`, expose it through `ros2 run`, add it to the launch flow, and verify:

```text
speech_text -> LLM -> llm_response -> ElevenLabs -> speaker
```

## Phase 4: connect microphone input

Finally launch Azure speech recognition and verify the complete loop.

This ordering makes debugging much easier because each layer can be tested independently.

---

# 7. Recommended immediate cleanup plan

I would make these changes first:

1. Delete the duplicate `src/colman_gazebo/colman_bringup` tree.
2. Add missing dependencies to `colman_motion/package.xml`.
3. Fix the 12 Ruff issues.
4. Install Gazebo models from `colman_gazebo/setup.py`.
5. Rename or compatibly deprecate the misleading Azure `TTS` environment variables.
6. Decide whether `calib_samples.npz` is a required fixture.
7. Keep the architecture diagram synchronized with the actual implementation; the current diagram correctly marks the AI, response topic, and arm command server as planned.
8. Package `colman_audio_output` and expose the existing ElevenLabs node through `ros2 run`.
9. Add `colman_interfaces`, an arm command server, and `colman_llm`.
10. Keep the LLM tool list very small and allow-listed.
11. Keep the local `llm` branch only if it is intentionally reserved for this work; otherwise it is redundant with `main`.

The current build result is good:

```text
11 packages finished
```

The current lint result needs cleanup, but the failures are mostly mechanical rather than architectural.