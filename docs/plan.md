# LLM integration plan

Design for adding voice-driven control to the robot. For how the rest of the repository works, see [`repository_summary.md`](repository_summary.md).

## Starting point

- `colman_audio_input` publishes recognized speech on `/speech_text`.
- `colman_audio_output` (open PR #8) subscribes to `llm_response` and speaks it through ElevenLabs.
- `colman_interfaces` and a motion action server (open PR #14) provide `MoveTo`, `Pick` and `Place` actions on `/move_to`, `/pick` and `/place`.
- There is no `colman_llm` package yet.

The `Pick` action takes a `frame` (for example `tag_3`) and reports a `stage` as feedback. The `Place` action takes a `location` frame and an optional `position`.

## Recommended initial `colman_llm` design

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

The `Pick` and `Place` actions from PR #14 can be reused internally, but the LLM should initially see one tool.

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

The ElevenLabs node is packaged as `colman_audio_output` in PR #8. The node should:

- subscribe to `llm_response`
- convert text to speech
- play it
- publish `tts_speaking` while playback is active

The node already supports stream, play, and save modes. For an initial version, use one provider and one mode. Do not add interruption, caching, and multi-provider fallback until the basic loop works. The `tts_speaking` topic can later coordinate microphone muting, but it should not be required for the first closed-loop test.

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
