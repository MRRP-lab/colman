# Colman (Collaborative Manipulation)

This is the repository for the Collaborative Manipulation project.

## Setup

Install pixi, and compile everything with the following:

```
curl -fsSL https://pixi.sh/install.sh | sh
pixi run build
```

## Run

Once you've compiled everything, open up gazebo with any of the following:

```
pixi run sim
```

You can see a full list of pixi tasks by running:

```
pixi task list
```

Common tasks:

| Task | What it does |
|---|---|
| `sim`, `sim-depth`, `sim-yolo` | Gazebo simulation (optionally with depth or YOLO) |
| `mock` | UR3e driver with mock hardware |
| `driver` | Driver supervisor that recovers the UR arm in place |
| `motion` | `MoveTo` / `Pick` / `Place` action server |
| `demo-turns`, `demo-choose`, `demo-circle` | Demo behaviours |
| `stop`, `resume` | Stop the arm / allow goals again |
| `speak` | Local text-to-speech node (Kokoro-82M) |
| `llm` | LLM node: `/speech_text` -> Claude -> `llm_response` (needs `ANTHROPIC_API_KEY`) |
| `fmt`, `lint` | `ruff` format and check |

## Voice

Speech runs locally where possible. The pipeline is
`microphone -> /speech_text -> LLM -> llm_response -> speaker`.
See [`docs/plan.md`](docs/plan.md) for the LLM design. The LLM node answers with Claude. Its main tool is `stack_cube`, which picks loose cubes of one colour and stacks them on that colour's base cube using `/pick` and `/place` goals to the motion server (`pixi run motion`). Tags 1 to 4 are the white cubes and 5 to 8 the blue cubes; tags 1 and 5 are the bases. See the Cube stacking demo section of [`docs/plan.md`](docs/plan.md). Pass `-p dry_run:=true` to `llm_node` to get canned tool results without an arm. Saying "stop", "halt" or "freeze" stops the arm at once (`/stop`) and cancels the running goal. Say "resume" to let it move again. A physical emergency stop remains the real safety mechanism.

- **Speech output** (`colman_audio_output`): `pixi run speak` starts a node that speaks
  text published on `llm_response` using [Kokoro-82M](https://huggingface.co/hexgrad/Kokoro-82M)
  (Apache 2.0). It needs no API key. Model weights download on first run, and it publishes
  `tts_speaking` while audio plays. Try it with:

  ```
  ros2 topic pub --once llm_response std_msgs/msg/String "{data: 'Hello'}"
  ```

- **LLM** (`colman_llm`): `pixi run llm` starts a node that sends each `/speech_text` message
  to Claude and publishes the reply on `llm_response`. It needs `ANTHROPIC_API_KEY`, from the
  environment or the repository's `.env` file, and exits at startup without it. Test it without a
  microphone (with `pixi run speak` running, the reply is spoken):

  ```
  ros2 topic pub --once /speech_text std_msgs/msg/String "{data: 'Hello'}"
  ```

- **Speech input** (`colman_audio_input`): `ros2 run colman_audio_input speech_to_text` publishes
  recognized speech on `/speech_text` using NVIDIA Parakeet, locally and without an API key. It
  mutes the microphone while the robot speaks (`tts_speaking`). Model weights download on first
  run. See [`src/colman_audio_input/README.md`](src/colman_audio_input/README.md) for parameters
  such as `input_device`.

### Voice-only mode

Talk to the robot with the microphone and speaker. This runs speech input, the LLM and speech
output together, with no arm or simulation. It needs `ANTHROPIC_API_KEY` set in the
repository's `.env` file (gitignored) or in your environment:

```
echo 'ANTHROPIC_API_KEY=sk-ant-...' >> .env
```

Build once with `pixi run build`, then start all three nodes. `dry_run` is on, so tool calls get canned results and no arm is needed. Ctrl-C stops them all:

```
pixi run bash -c 'trap "kill 0" EXIT; ros2 run colman_audio_output kokoro_tts & ros2 run colman_llm llm_node --ros-args -p dry_run:=true & ros2 run colman_audio_input speech_to_text'
```

Notes:

- The microphone is muted while the robot speaks, so you cannot interrupt it.
- The LLM node replies to everything the microphone hears. Use a quiet room or a headset.
- If nodes from an earlier run are still alive, you get duplicate replies. Clear them first with
  `pkill -f 'llm_node|kokoro_tts|speech_to_text'`.
- If a node fails to start with a `StopIteration` error, its package is not built. Run
  `pixi run build`.

For more detail on how the packages fit together, see
[`docs/repository_summary.md`](docs/repository_summary.md).

## Coval LEM Series Vacuum Pump

[Documentation](https://doc.coval.com/g/LEM/doc/lem_doc_coval_2022_v06_us.pdf)
[Operating Instructions](https://doc.coval.com/g/LEM/not/lem_notice_coval_v7a_2023_01.pdf)
