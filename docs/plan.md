# LLM integration plan

Design for adding voice-driven control to the robot. For how the rest of the repository works, see [`repository_summary.md`](repository_summary.md).

## Starting point

- `colman_audio_input` publishes recognized speech on `/speech_text` (`std_msgs/String`).
- `colman_audio_output` subscribes to `llm_response` (`std_msgs/String`), speaks it locally with Kokoro-82M, and publishes `tts_speaking` (`std_msgs/Bool`, transient-local) while audio plays.
- `colman_audio_input` on the `NVIDIA-Parakeet` branch is the Parakeet speech recognition node (`onnx-asr`, `silero-vad`, `sounddevice`, all in `pixi.toml`). It already mutes the microphone while `tts_speaking` is true plus a 600 ms tail (`mute_topic`, `mute_tail_ms` parameters), and `test/wav_file_check.py` runs it on a WAV file. That branch is not merged yet, and `main` still has the Azure node. Merge `NVIDIA-Parakeet` before the closed-loop test.
- `colman_interfaces` and `colman_motion` provide `MoveTo`, `Pick` and `Place` actions on `/move_to`, `/pick` and `/place`, plus `std_srvs/Trigger` services `/stop` and `/resume`. Run the server with `pixi run motion`.
- `colman_bringup` has a driver supervisor (`pixi run driver`) that recovers the UR arm in place.
- There is no `colman_llm` package yet.

The `Pick` action takes a `frame` (for example `tag_3`) and reports a `stage` as feedback. The `Place` action takes a `location` frame and an optional `position`. Both return `success` and `message`.

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
    executor.py        # validates a tool call and runs the ROS action
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

System prompt, kept short and static (this also makes it cacheable):

- You control a robot arm through the provided tools.
- Only call a tool when the user clearly asks the arm to do something.
- Answer in one or two short spoken sentences, with no markdown, lists or emoji.
- If a request is ambiguous or names something not in the tool's list, ask a question instead of guessing.
- Report tool failures plainly.

Inject the live list of known objects and destinations into the system prompt from the allow-lists, so the two never drift apart.

## Tool definitions

The tools are added in separate steps (see Order of work): `pick_and_place` first, then `stop_robot`, then `resume_robot`. Constrain arguments with `enum` so Claude can only pick known names, instead of validating free text afterwards:

```python
PICK_AND_PLACE = {
    "name": "pick_and_place",
    "description": (
        "Pick up one known object with the arm and place it at a known "
        "destination. Takes about a minute and blocks other arm commands."
    ),
    "strict": True,
    "input_schema": {
        "type": "object",
        "properties": {
            "object": {"type": "string", "enum": list(OBJECT_FRAMES)},
            "destination": {"type": "string", "enum": list(DESTINATION_FRAMES)},
        },
        "required": ["object", "destination"],
        "additionalProperties": False,
    },
}
```

The enum values are human names. The node maps them to frames, so Claude never sees or sends a raw frame name:

```python
# Placeholder tag names. Replace them when the real tags are named.
OBJECT_FRAMES = {
    "red block": "tag_3",
    "blue block": "tag_5",
}

DESTINATION_FRAMES = {
    "left tray": "tag_1",
    "right tray": "tag_6",
}
```

Even with `strict: true` the executor re-checks every value against these dicts before sending a goal, because the node is the safety boundary.

Never let Claude supply arbitrary joint positions, Cartesian coordinates, ROS topic, service or action names, or code.

`stop_robot` calls `/stop` and takes no arguments. `resume_robot` calls `/resume` and takes no arguments. Both are added after `pick_and_place`, each in its own step. A spoken stop must always work (see Safety).

Parse `tool_use.input` as already-decoded JSON from the SDK. Do not string-match the serialized form.

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

- **A spoken stop must interrupt a running action.** With a single blocking worker, "stop" would sit in the queue behind the pick-and-place it is meant to stop. Handle stop words in the subscription callback itself, before queueing. Match the whole words `stop`, `halt` and `freeze`, case-insensitive. This accepts false matches such as "don't stop at the red block", which is the safe direction. On a match, call `/stop`, cancel the active goal, drop the queue, and publish "Stopped." on `llm_response`. `stop_robot` also exists as a tool for stops phrased differently. A physical emergency stop remains the real safety mechanism. Voice stop is a convenience on top.
- After `/stop` the server rejects goals until `/resume`. Resuming works both ways: spoken, through the `resume_robot` tool, and manual, with `pixi run resume`. The spoken reply to a rejected goal says the arm is stopped and must be resumed.
- Speech is not ignored while the arm moves. Goals are serialized by the single worker, and a tool call while the arm reports busy returns a `tool_result` error with `is_error: true`.
- Rate-limit tool calls (at most one pick-and-place at a time) and log each tool call with its validated arguments.
- Treat transcription as untrusted input. A misheard sentence can still only select from the enumerated objects and destinations.

## Audio loop

The output node uses its defaults: `stream` mode and the `af_heart` voice, with a bounded queue of 5.

The microphone mute is already implemented in the Parakeet node on the `NVIDIA-Parakeet` branch (see Starting point). It subscribes to `tts_speaking` with transient-local QoS, so a late-starting mic node sees the current state. Merge that branch before the closed-loop test.

Before a long action starts, the worker publishes the acknowledgement "On it." as a separate `llm_response` message, so the user knows the command was heard.

The speech loop:

```text
User: "Pick up the red block and put it in the left tray."
    |
    v
/speech_text
    |
    v
Claude returns tool_use: pick_and_place(object="red block", destination="left tray")
    |
    v
node validates -> /pick tag_3 -> /place tag_1
    |
    v
Claude receives success/failure as tool_result
    |
    v
/llm_response: "I picked up the red block and placed it in the left tray."
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
4. Closed loop with real audio, after `NVIDIA-Parakeet` is merged so the mic mute is present.

## Order of work

1. Scaffold `colman_llm` with a node that echoes `/speech_text` to `llm_response` through a Claude call with no tools. Include the `ANTHROPIC_API_KEY` check, `package.xml` with `python3-anthropic`, and a `pixi run llm` task.
2. Add the `pick_and_place` tool with a fake executor that returns canned results.
3. Wire the executor to `/pick` and `/place`.
4. Add the spoken stop fast path and the `stop_robot` tool.
5. Add the `resume_robot` tool.
6. Merge `NVIDIA-Parakeet` for the microphone and its mute on `tts_speaking`.
7. Tune the system prompt against a list of real spoken phrases.
