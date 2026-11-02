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
| `listen` | Local speech-to-text node (Parakeet) |
| `llm` | Claude node: speech text in, arm tool calls and a spoken reply out |
| `speak` | Local text-to-speech node (Kokoro-82M) |
| `fmt`, `lint` | `ruff` format and check |

## Voice

Talk to the robot: `microphone -> /speech_text -> LLM -> move_to action + llm_response -> speaker`.
See [`docs/plan.md`](docs/plan.md) for the design.

- **Speech input** (`colman_audio_input`): `pixi run listen` transcribes the microphone
  with [NVIDIA Parakeet](https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2) and publishes
  `/speech_text`. It runs locally and mutes itself while the robot speaks. Model weights
  download on first run. The default audio device may reject 16 kHz, so pick the
  microphone by name or index:

  ```
  pixi run ros2 run colman_audio_input speech_to_text --ros-args -p input_device:=<name or index>
  ```

- **LLM** (`colman_llm`): `pixi run llm` sends each utterance to Claude Sonnet 5.5. Claude
  has one tool, `move_arm`, which sends a `MoveTo` goal for "position one" (`Look1`) or
  "position two" (`Compact`). Speech that is not aimed at the robot is ignored. Put the key
  in `.env` at the repository root as `ANTHROPIC_API_KEY=...` (or export it).
- **Speech output** (`colman_audio_output`): `pixi run speak` speaks text published on
  `llm_response` with [Kokoro-82M](https://huggingface.co/hexgrad/Kokoro-82M) (Apache 2.0). It
  needs no API key, and publishes `tts_speaking` while audio plays.

To run the whole loop on the real arm, build, then start each of these in its own terminal
(or in the background):

```
pixi run build
pixi run driver     # arm driver; wait for "controllers are active"
pixi run motion     # motion server; wait for "action servers started"
pixi run listen     # microphone (add the input_device argument above if needed)
pixi run llm
pixi run speak
```

Then say "move the arm to position one" or "position two". To skip the microphone, publish
the text instead:

```
pixi run ros2 topic pub --once /speech_text std_msgs/msg/String "{data: 'move the arm to position one'}"
```

Run `pixi run stop` (or use the pendant e-stop) to halt the arm. There is no spoken stop and no
wake word yet. Anything the microphone hears goes to Claude.

For more detail on how the packages fit together, see
[`docs/repository_summary.md`](docs/repository_summary.md).

## Coval LEM Series Vacuum Pump

[Documentation](https://doc.coval.com/g/LEM/doc/lem_doc_coval_2022_v06_us.pdf)
[Operating Instructions](https://doc.coval.com/g/LEM/not/lem_notice_coval_v7a_2023_01.pdf)
