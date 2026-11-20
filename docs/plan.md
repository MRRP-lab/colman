# LLM integration plan

Design for adding voice-driven control to the robot. For how the rest of the repository works, see [`repository_summary.md`](repository_summary.md).

## Starting point

- `colman_audio_input` publishes recognized speech on `/speech_text` (`std_msgs/String`).
- `colman_audio_output` subscribes to `llm_response` (`std_msgs/String`), speaks it locally with Kokoro-82M, and publishes `tts_speaking` (`std_msgs/Bool`, transient-local) while audio plays.
- `colman_audio_input` is the Parakeet speech recognition node (`onnx-asr`, `silero-vad`, `sounddevice`, all in `pixi.toml`), merged from `NVIDIA-Parakeet`. It mutes the microphone while `tts_speaking` is true plus a 600 ms tail (`mute_topic`, `mute_tail_ms` parameters), and `test/wav_file_check.py` runs it on a WAV file.
- `colman_interfaces` and `colman_motion` provide `MoveTo`, `Pick` and `Place` actions on `/move_to`, `/pick` and `/place`, plus `std_srvs/Trigger` services `/stop` and `/resume`. Run the server with `pixi run motion`.
- `colman_bringup` has a driver supervisor (`pixi run driver`) that recovers the UR arm in place.
- `colman_llm` exists and every step of Order of work is done: the node answers `/speech_text` through Claude, the `pick_and_place` tool sends `/pick` then `/place` goals, a spoken stop word calls `/stop` and cancels the active goal before anything is queued, and the `stop_robot` and `resume_robot` tools call `/stop` and `/resume`. A `dry_run` parameter swaps in a fake executor with canned results for use without an arm. Claude is offered `stack_cube`, `pick_and_place`, `go_to_pose`, `release_cube`, `report_state`, `stop_robot` and `resume_robot`; see Cube stacking demo.
- `colman_perception` has a second AprilTag detector for the fixed overhead camera and a `world_state` node that fuses both detectors into `tag_N` frames and the `/world_state` topic (`colman_interfaces/msg/WorldState`): which cube is on which, which cubes are hidden by the arm, and whether the overhead camera is alive. The LLM node and the motion server both plan from it.
- `pixi run stack` starts the whole pipeline (driver, cameras, detectors, `world_state`, motion server, Kokoro, Parakeet and `llm_node`); `pixi run stack-dry` runs it without the arm driver and with `dry_run:=true`.

The `Pick` action takes a `frame` (for example `tag_3`) and an optional `place_location`, and reports a `stage` as feedback. The `Place` action takes a `location` frame and an optional `position`. Both return `success` and `message`.

Behaviours of the motion server that the LLM node must respect:

- It accepts **one goal at a time**. A goal sent while the arm is busy, stopped or shutting down is rejected immediately.
- After `/stop` it rejects every goal until `/resume` is called.
- Cancelling a goal halts the arm.

## Design

Use the Anthropic Python SDK (`anthropic`) with **Claude Sonnet 5.5** (`claude-sonnet-5-5`) for both conversation and tool selection. Claude decides when a spoken request means "move the arm" by returning a `tool_use` block. The node validates that call and translates it into a ROS action goal. Claude never talks to ROS directly.

```text
microphone
   |
   v
colman_audio_input
   |  /speech_text
   v
colman_llm  --- messages.create(tools=[...]) ---> Claude Sonnet 5.5
   |   ^                                               |
   |   +------------ tool_use / end_turn -------------+
   |
   +--> /pick and /place action clients (validated goals only)
   |
   |  llm_response
   v
colman_audio_output / Kokoro-82M
   |
   v
speaker
```

Keep the implementation to one node first. Do not build a general-purpose agent, and do not add a second provider. The Anthropic call lives in one small function.

The API key comes from the `ANTHROPIC_API_KEY` environment variable. The node raises a `RuntimeError` at startup and exits if it is not set.

## Package layout

```text
src/colman_llm/
  colman_llm/
    __init__.py
    llm_node.py        # ROS node: subscription, worker thread, publishing
    claude_client.py   # Anthropic SDK wrapper: one turn in, text or tool calls out
    tools.py           # tool schemas and allow-lists
    executor.py        # validates a tool call and runs it
    action_call.py     # one blocking ROS action call, with cancel
    service_call.py    # one blocking Trigger service call
    stop_words.py      # whole-word stop/halt/freeze match
    world_view.py      # what the camera says is on the table (from /world_state)
    world_watch.py     # notices when the table changes by itself, between requests
    fake_arm.py        # pretend arm and table for dry run and tests
  resource/colman_llm
  package.xml
  setup.py
  test/
```

Splitting out `claude_client.py` and `executor.py` lets the conversation loop be unit-tested with a fake client and a fake action client, without ROS or an API key.

## Model call

Use the manual loop (`client.messages.create`), not the SDK tool runner. The tool handlers block on ROS actions, and the node needs to control cancellation, timeouts and history itself.

Settings:

- `model="claude-sonnet-5-5"`, exposed as a ROS parameter.
- `max_tokens=1024`. Replies are spoken aloud and should be short.
- `output_config={"effort": "low"}`. This is a voice assistant, so latency matters more than depth.
- `thinking`: leave it at the default (adaptive). Do not send `{"type": "disabled"}` or `budget_tokens`, which return a 400 on this model.
- **No forced `tool_choice`.** `any` and `tool` return a 400 on Sonnet 5.5. Leave `tool_choice` as `auto` and tell Claude in the system prompt when to use the tool.
- No `temperature`, `top_p` or `top_k`. Non-default values return a 400.
- Set `strict: true` on each tool and `additionalProperties: false` in its schema, so arguments always match the schema.
- Use non-streaming calls. With `max_tokens=1024` there is no timeout risk.
- Refusals: always check `stop_reason`. On `"refusal"`, say a short fixed line ("I can't help with that.") instead of reading `content`.
- Preserved thinking: thinking blocks in the history must be replayed unchanged. Append `response.content` to the history as-is, and never edit or reorder earlier turns. Trim history only by removing whole oldest exchanges together.

System prompt, static (the live cube list is appended from the allow-lists):

- You are the voice of a robot arm that stacks cubes, and you hear raw speech-to-text from a busy room. Reply with exactly the word `IGNORE` to anything that is not clearly a command or question about the arm or the cubes. `ClaudeClient.say` turns that into silence and keeps it out of the history.
- Only call a tool when the user clearly asks the arm to do something.
- Before `stack_cube`, `pick_and_place` or `go_to_pose`, say one short sentence in your own words about what you are about to do, because the arm takes a while. After the result, say only what the user still needs to know.
- Answer in at most two short spoken sentences, with no markdown, lists or emoji.
- If a request is ambiguous or names something not in the tool's list, ask one short question instead of guessing.
- Report tool failures plainly.

Inject the live list of known objects and destinations into the system prompt from the allow-lists, so the two never drift apart.

## Tool definitions

The tools are added in separate steps (see Order of work): `pick_and_place` first, then `stop_robot`, then `resume_robot`. Constrain arguments with `enum` so Claude can only pick known names, instead of validating free text afterwards:

```python
PICK_AND_PLACE = {
    "name": "pick_and_place",
    "description": (
        "Pick up one cube and place it on top of another cube. Takes about a "
        "minute and blocks other arm commands."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {
            "object": {"type": "string", "enum": list(CUBE_NAMES)},
            "destination": {"type": "string", "enum": list(CUBE_NAMES)},
        },
        "required": ["object", "destination"],
        "additionalProperties": False,
    },
}
```

The enum values are human names. The node maps them to frames, so Claude never sees or sends a raw frame name:

```python
# The eight tagged cubes: tags 1 to 4 are white cubes 1 to 4, tags 5 to 8 blue cubes 5 to 8.
# No cube is special: any cube can be picked and any cube can be placed on.
CUBE_NAMES = {"white cube 1": "tag_1", "white cube 2": "tag_2", ..., "blue cube 8": "tag_8"}

POSES = {"home": "home", "compact": "Compact"}
```

`pick_and_place(object, destination)` picks one cube and places it on top of any other cube, in any colour. Claude never names a tag; the node maps the spoken names above to frames. The node also checks the move against the latest `/world_state` (what the overhead camera sees) before the arm moves, and again after the pick, and refuses with a plain-language reason if the cube has a cube on top of it, is already on the destination, or the destination is covered. It also passes the destination to the motion server as the `Pick` goal's `place_location`, and the server refuses the pick before it moves if it cannot place there (see Cube stacking demo), so a cube is never picked up that cannot then be put down. There is no height limit in the node; that check is what stops a stack too tall or too far away (see `docs/arm_reach`). It then waits for the camera to see the cube on the destination before reporting success.

Even with `strict: true` the executor re-checks every value against these dicts before sending a goal, because the node is the safety boundary.

Never let Claude supply arbitrary joint positions, Cartesian coordinates, ROS topic, service or action names, or code.

The other tools:

- `release_cube` calls `/release` (vacuum off) and takes no arguments. It is refused while the arm is busy moving. If the node believed the arm was holding a cube, that cube is recorded as loose on the table.
- `go_to_pose(pose)` calls the `/move_to` action with one of the named poses in `POSES`. It is refused while the arm is busy or holding a cube. `Zero` is left out on purpose, and raw joint or Cartesian targets are never accepted.
- `report_state` takes no arguments and never moves the arm. It reports the stacks, the loose cubes, what the node believes the arm is holding, and whether it is stopped. The stopped flag follows `stop_robot`, `resume_robot` and a spoken stop; a manual `pixi run resume` is not seen by the node.

`stop_robot` calls `/stop` and takes no arguments. `resume_robot` calls `/resume` and takes no arguments. Both are added after `pick_and_place`, each in its own step. A spoken stop must always work (see Safety).

Parse `tool_use.input` as already-decoded JSON from the SDK. Do not string-match the serialized form.

## Cube stacking demo

The demo uses tagged cubes. Tags 1 to 4 are white cubes and tags 5 to 8 are blue cubes. No cube is a base: any cube can be moved and any cube can go on any other. The detector config (`colman_perception/config/apriltag.yaml`) registers IDs 1 to 8 at 39.2 mm.

Claude gets `stack_cube(color, count)` for building a stack, with `color` in `white` or `blue` and `count` from 2 to 4, the number of cubes in the finished stack, and `pick_and_place`, `go_to_pose`, `release_cube`, `report_state`, `stop_robot` and `resume_robot` (see Tool definitions). The node, not Claude, decides which cube moves where:

- The stacks come from the `world_state` node, which reads them off the overhead camera every frame; nothing remembers the arm's own moves. The height of a column is `round(corrected z / 50 mm)` of its top tag (`overhead_correction.py` fixes the overhead depth bias), tags at the same x, y form a column, a tag covered by a taller tag at the same x, y is remembered, a tag that vanishes with nothing over it and the arm out of the way is dropped after 10 s (`LOST_S`), and a cube the arm is hiding is remembered for up to 30 minutes (`ARM_HIDE_MAX_S`), because the arm can stay parked over a stack between commands and a covered cube cannot move. If what the camera sees contradicts what was remembered (a cube pulled from under a stack) the camera wins and the order is marked uncertain. `stack_cube` builds on the tallest column made only of that colour, whichever cube is at its bottom, and adds the top cube of the shortest other column of that colour, so loose cubes go first. Each is picked with `/pick` and placed with `/place` on the top of the stack. Columns that hold another colour, or a cube the camera cannot identify, are left alone. `/place` with a `location` already drops 50 mm above the tag, which is one cube height, so the cube lands on top of the cube carrying that tag.
- Before a pick the motion server checks the place. `Pick` has an optional `place_location`, the frame of the cube this one will go on. The server finds that tag, then solves the same drop pose (one cube above the tag) and approach pose (0.05 m higher) that `/place` will, with the cube modelled on the tip and the destination column left out of the scene, using `vertical_pair` (MoveIt/TRAC-IK, collision checked, all four quarter turns). Nothing moves. If there is no solution the goal fails at once with "No safe arm pose to place on `<tag>`, so `<tag>` was not picked up". The cube's yaw in the tool is not known until it is grasped, so the squaring turn `/place` adds afterwards is left out of the check. `stack_cube` stops at that refusal and reports it.
- Every request carries the table. `ClaudeClient` adds the current `/world_state` (and what the arm holds) as a system block, rebuilt for each request and never kept in the history, so Claude answers from what the camera sees.
- `world_watch.py` speaks up between requests. When a change lasts 3 s (a cube lost or found, one moved by someone, the camera lost or back) it publishes a sentence on `llm_response`. It stays quiet while a tool call runs, the arm holds a cube or an utterance is queued or being handled, takes a baseline only after the table has been still for 3 s, spaces announcements 10 s apart, and what it said joins the history so a follow-up like "put it back" makes sense. The `announce_changes` parameter (default true) turns it off.
- After a `pick_and_place` or `stack_cube` job the executor moves the arm to the `home` pose (`clear_view_pose`), because the overhead camera cannot see a cube the arm covers and cubes next to where the job ended drop out of view. It is skipped while a cube is held or the arm is stopped, and if another job has already started.
- There is no height limit in the code: not in the node, the executor, `world_state` or the stack model. How tall a stack can be is set by the arm's reach, which the motion server checks with the place check above before a pick.
- A stack that is already tall enough is reported, not rebuilt, and a count the free cubes cannot reach stops and says so. A failed pick or place, or a spoken stop, ends the run and reports how many were stacked. A failed cube is not counted, so a retry targets the same cube and spot.
- The state is in memory. Restarting the node resets it, and the stacks are read from the camera, so a restart loses nothing.
- `pick_and_place` and `stack_cube` share one model of which cube sits on which, so a cube moved by one is where the other expects it. A cube left in the gripper after a failed place blocks every other arm tool until `release_cube`.

If a place fails after a successful pick, the vacuum still holds the cube and the next pick fails with "Already holding". Release it with `ros2 service call /release std_srvs/srv/Trigger`.

## Pick and place: two actions, one tool

Do not add a `PickAndPlace.action` first. The `pick_and_place` tool handler can call `/pick` then `/place` from the worker thread. It aborts and reports if `/pick` fails, and it only calls `/place` after `/pick` succeeds. This avoids new interface code in `colman_interfaces` and a new server path to test.

`pick_and_place` is built first. `stop_robot` and `resume_robot` are separate later steps.

## Conversation loop and threading

The Claude API and the arm can each take seconds to minutes. The `/speech_text` callback must only enqueue.

```text
ROS subscription callback
    -> put text on a bounded queue
    -> worker thread takes one utterance at a time
    -> worker calls Claude (messages.create)
    -> if stop_reason == "tool_use": validate, run ROS action, wait for result,
       append tool_result, call Claude again (loop, max 5 rounds)
    -> worker publishes the final text on llm_response
```

Details:

- Use a `MultiThreadedExecutor` and a `ReentrantCallbackGroup`. Call `rclpy.action.ActionClient.send_goal_async` and wait on a `threading.Event` in the worker thread. Never spin the node from the worker thread.
- Use a **single worker**. Utterances are handled in order, which also matches the server's one-goal-at-a-time rule.
- Cap the tool loop at 5 rounds per utterance.
- Return all `tool_result` blocks for one assistant turn in a **single** user message, and match every `tool_use_id`. Return failures as `tool_result` with `is_error: true` and a plain-language message.
- Every action wait has a timeout, and the goal is cancelled on timeout. The timeouts are ROS parameters: `pick_timeout_s` (default 120) and `place_timeout_s` (default 120).
- Catch SDK errors in a chain, most specific first: `RateLimitError`, `APIConnectionError`, `APIStatusError`. The SDK already retries 429 and 5xx twice. On a final failure, publish a short fallback line ("I couldn't reach my brain just now.") so the user is not left in silence.
- Log `response._request_id` and `usage` for each call.

## Safety

- **A spoken stop must interrupt a running action.** With a single blocking worker, "stop" would sit in the queue behind the pick-and-place it is meant to stop. Handle stop words in the subscription callback itself, before queueing. Match the whole words and phrases in `stop_words.py` (`stop`, `halt`, `freeze`, `cancel`, `abort`, `hold still`, `stay still`, `don't move`, `that's enough`), case-insensitive. This accepts false matches such as "don't stop at the red block", which is the safe direction. On a match, call `/stop`, cancel the active goal, drop the queue, and publish "Stopped." on `llm_response`. The reply to any utterance that was in flight when the stop arrived is dropped, so the user hears only "Stopped.". The microphone is muted while the robot speaks, so a stop word spoken while the robot is saying what it is about to do is not heard; the physical emergency stop covers that gap. `stop_robot` also exists as a tool for stops phrased differently. A physical emergency stop remains the real safety mechanism. Voice stop is a convenience on top.
- After `/stop` the server rejects goals until `/resume`. Resuming works both ways: spoken, through the `resume_robot` tool, and manual, with `pixi run resume`. The spoken reply to a rejected goal says the arm is stopped and must be resumed.
- Speech is not ignored while the arm moves. Goals are serialized by the single worker, and a tool call while the arm reports busy returns a `tool_result` error with `is_error: true`.
- Rate-limit tool calls (at most one pick-and-place at a time) and log each tool call with its validated arguments.
- Treat transcription as untrusted input. A misheard sentence can still only select from the enumerated objects and destinations.

## Audio loop

The output node uses its defaults: `stream` mode and the `bm_fable` voice (British), with a bounded queue of 5.

The microphone mute is implemented in the Parakeet node (see Starting point). It subscribes to `tts_speaking` with transient-local QoS, so a late-starting mic node sees the current state.

There is no fixed acknowledgement. The system prompt tells Claude to say one short sentence in its own words before `stack_cube`, `pick_and_place` or `go_to_pose`. `ClaudeClient.say` hands any text Claude writes before a tool call to the node as soon as it arrives, so it is spoken while the arm moves, and the reply returned afterwards is only what Claude says after the tool result. The one fixed phrase left is "Stopped." on a spoken stop, which skips Claude so it is instant.

The speech loop:

```text
User: "Put white cube three on blue cube one."
    |
    v
/speech_text
    |
    v
Claude returns tool_use: pick_and_place(object="white cube 3", destination="blue cube 5")
    |
    v
node validates -> /pick tag_3 -> /place tag_5
    |
    v
Claude receives success/failure as tool_result
    |
    v
/llm_response: "I picked up white cube 3 and placed it on blue cube 5."
    |
    v
Kokoro speaks the response
```


## Cost and latency

- The system prompt and tool list are small and static, so no `cache_control` is used. The log of `usage` for each call shows the token counts.
- Keep the last 5 exchanges in history, trimmed as whole exchanges. Voice commands rarely need long memory, and a smaller history means lower latency.
- Sonnet 5.5 is $2 / $10 per million tokens, so a short voice interaction costs a fraction of a cent. The dominant delay is the speech-to-text and text-to-speech round trips, not the model.

## Testing

1. Unit tests with a fake Anthropic client: plain reply, valid tool call, unknown object, malformed arguments, refusal, API error, tool loop cap.
2. Unit tests for the executor with a fake action client: pick fails, pick succeeds then place fails, timeout, busy rejection.
3. Simulation: run `colman_llm` with the motion server in Gazebo and publish text on `/speech_text` directly (`ros2 topic pub`), without the microphone.
4. Closed loop with real audio, using the Parakeet node and its mic mute.

## Order of work

1. (Done) Scaffold `colman_llm` with a node that echoes `/speech_text` to `llm_response` through a Claude call with no tools. Include the `ANTHROPIC_API_KEY` check, `package.xml` with `python3-anthropic`, and a `pixi run llm` task.
2. (Done) Add the `pick_and_place` tool with a fake executor that returns canned results.
3. (Done) Wire the executor to `/pick` and `/place`.
4. (Done) Add the spoken stop fast path and the `stop_robot` tool.
5. (Done) Add the `resume_robot` tool.
6. (Done) Check the microphone mute on `tts_speaking` in the closed loop. Parakeet is merged and mutes on `tts_speaking`. Checked live: with muting on, the robot's own sentence was not transcribed; with the mute topic pointed at an unused topic, the mic transcribed it.
7. (Done) Tune the system prompt against a list of real spoken phrases.
8. (Done) Add `go_to_pose`, `release_cube` and `report_state`.
9. (Done) Add `stack_cube`, with stacks read from the overhead camera through `world_state`.
10. (Done) Add `pixi run stack` and `stack-dry`, which start the whole pipeline.
11. (Done) Let any cube go on any other and let Claude say what it is doing, with `IGNORE` for speech that is not for the arm.
12. (Done) Check, in the motion server, that the cube can be placed on its destination before the pick.
13. (Done) Tell Claude what the table looks like in every request, and let it speak up when the table changes by itself.
14. (Done) Move the arm home after a job so it does not hide the cubes.
15. (Done) Choose a licence (Apache-2.0) and make YOLO optional.
16. (Done) Remove the cap on stack height from `world_state`.
