# LLM integration plan

Design for adding voice-driven control to the robot. For how the rest of the repository works, see [`repository_summary.md`](repository_summary.md).

## Starting point

- `colman_audio_input` publishes recognized speech on `/speech_text` (`std_msgs/String`).
- `colman_audio_output` subscribes to `llm_response` (`std_msgs/String`), speaks it locally with Kokoro-82M, and publishes `tts_speaking` (`std_msgs/Bool`, transient-local) while audio plays.
- `colman_audio_input` currently uses Azure. Dependencies for a local replacement (`onnx-asr`, `silero-vad`, `sounddevice`) are in `pixi.toml` but unused.
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

Keep the implementation to one node first. Do not build a general-purpose agent, and do not add a second provider. The Anthropic call lives in one small function so a different model or provider could replace it later.

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

- `model="claude-sonnet-5-5"`, exposed as a ROS parameter so it can be changed without code edits.
- `max_tokens=1024`. Replies are spoken aloud and should be short.
- `output_config={"effort": "low"}`. This is a voice assistant, so latency matters more than depth. Raise it to `medium` only if tool selection is unreliable in testing.
- `thinking`: leave it at the default (adaptive). Do not send `{"type": "disabled"}` or `budget_tokens`, which return a 400 on this model. If it needs to be off, use `{"type": "between_tools"}` at effort `high` or lower.
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

Expose one tool at first. Constrain arguments with `enum` so Claude can only pick known names, instead of validating free text afterwards:

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

A second tool worth adding early is `stop_robot`, which calls `/stop`. A spoken "stop" must always work (see Safety).

Parse `tool_use.input` as already-decoded JSON from the SDK. Do not string-match the serialized form.

## Pick and place: two actions, one tool

Do not add a `PickAndPlace.action` first. The `pick_and_place` tool handler can call `/pick` then `/place` from the worker thread. It aborts and reports if `/pick` fails, and it only calls `/place` after `/pick` succeeds. This avoids new interface code in `colman_interfaces` and a new server path to test.

Add `PickAndPlace.action` later only if the sequence needs to be atomic, for example to keep the arm from being claimed by another client between pick and place.

Later tools could include `move_to_named_pose`, `pick_object` and `place_object`.

## Conversation loop and threading

The Claude API and the arm can each take seconds to minutes. The `/speech_text` callback must only enqueue.

```text
ROS subscription callback
    -> put text on a bounded queue
    -> worker thread takes one utterance at a time
    -> worker calls Claude (messages.create)
    -> if stop_reason == "tool_use": validate, run ROS action, wait for result,
       append tool_result, call Claude again (loop, max N rounds)
    -> worker publishes the final text on llm_response
```

Details:

- Use a `MultiThreadedExecutor` and a `ReentrantCallbackGroup`, or `rclpy.action.ActionClient` with `send_goal_async` and a `threading.Event` / future wait on the worker thread. Never spin the node from the worker thread.
- Use a **single worker**. Utterances are handled in order, which also matches the server's one-goal-at-a-time rule.
- Cap the tool loop at about 5 rounds per utterance.
- Return all `tool_result` blocks for one assistant turn in a **single** user message, and match every `tool_use_id`. Return failures as `tool_result` with `is_error: true` and a plain-language message.
- Put a timeout on every action wait, and cancel the goal on timeout.
- Catch SDK errors in a chain, most specific first: `RateLimitError`, `APIConnectionError`, `APIStatusError`. The SDK already retries 429 and 5xx twice. On a final failure, publish a short fallback line ("I couldn't reach my brain just now.") so the user is not left in silence.
- Log `response._request_id` and `usage` for each call.

## Safety

- **A spoken stop must interrupt a running action.** With a single blocking worker, "stop" would sit in the queue behind the pick-and-place it is meant to stop. Handle stop words in the subscription callback itself (match `stop`, `halt`, `freeze`), before queueing: call `/stop`, cancel the active goal, and drop the queue. A physical emergency stop remains the real safety mechanism. Voice stop is a convenience on top.
- After `/stop` the server rejects goals until `/resume`. Decide whether resuming is a spoken command or a manual step. A manual step is safer for the first version. Say so in the spoken reply to a rejected goal.
- Ignore speech while the arm is moving? Not needed if goals are serialized, but reject new tool calls with a clear `tool_result` error if the arm reports busy.
- Rate-limit tool calls (at most one pick-and-place at a time) and log each tool call with its validated arguments.
- Treat transcription as untrusted input. A misheard sentence can still only select from the enumerated objects and destinations.
- Test with the arm in simulation or with the vacuum gripper disabled first.

## Audio loop

The output node already supports stream, play and save modes and a bounded queue. For the first version use one mode and one voice.

**The microphone does not currently use `tts_speaking`.** `speech_to_text_node.py` (Azure) has no mute logic, so the robot will hear its own voice through the speaker and answer itself. Fix this before the first closed-loop test, not later. Either:

- subscribe to `tts_speaking` in `colman_audio_input` and drop recognized text while it is `True` (plus a short tail of about 0.5 s), or
- do the same drop in `colman_llm`, which keeps `colman_audio_input` unchanged.

If speech input moves to a local recognizer, put the mute in that node. The first option is cleaner. The transient-local QoS means a late-starting mic node still sees the current state.

Also consider speaking a short acknowledgement (for example "On it.") before a long action starts, so the user knows the command was heard. It can be published as a separate `llm_response` message from the worker.

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

## Credentials and dependencies

Set the key in the environment:

```bash
export ANTHROPIC_API_KEY="..."
```

`anthropic.Anthropic()` reads it automatically, so the node never handles the key. Do not put it in `pixi.toml`, launch files, source code, YAML files or git history. Follow the audio output node's pattern of naming the environment variable in a parameter if it needs to be configurable, and fail at startup with a clear message when the key is missing.

Add `anthropic` through `pixi.toml` (`[pypi-dependencies]`, with a version range) and `package.xml` rather than installing it ad hoc. Note that `anthropic` 1.x is built on `httpx2`, so avoid importing `httpx` directly.

## Cost and latency

- The system prompt and tool list are small and static. Add `cache_control={"type": "ephemeral"}` on the request only if the system prompt grows beyond the minimum cacheable size. Check `usage.cache_read_input_tokens` before relying on it.
- Keep conversation history short: the last few exchanges are enough. Voice commands rarely need long memory, and a smaller history means lower latency.
- Sonnet 5.5 is $2 / $10 per million tokens, so a short voice interaction costs a fraction of a cent. The dominant delay is the speech-to-text and text-to-speech round trips, not the model.

## Testing

1. Unit tests with a fake Anthropic client: plain reply, valid tool call, unknown object, malformed arguments, refusal, API error, tool loop cap.
2. Unit tests for the executor with a fake action client: pick fails, pick succeeds then place fails, timeout, busy rejection.
3. Simulation: run `colman_llm` with the motion server in Gazebo and publish text on `/speech_text` directly (`ros2 topic pub`), without the microphone.
4. Closed loop with real audio, only after the mic mute is in place.

## Suggested order of work

1. Scaffold `colman_llm` with a node that echoes `/speech_text` to `llm_response` through a Claude call with no tools.
2. Add the `pick_and_place` tool with a fake executor that returns canned results.
3. Wire the executor to `/pick` and `/place`.
4. Add the voice `stop` path and `stop_robot` tool.
5. Add the mic mute on `tts_speaking`.
6. Tune the system prompt and effort level against a list of real spoken phrases.
