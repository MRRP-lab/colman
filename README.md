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
| `fmt`, `lint` | `ruff` format and check |

## Voice

Speech runs locally where possible. The planned pipeline is
`microphone -> /speech_text -> LLM -> llm_response -> speaker`.
See [`docs/plan.md`](docs/plan.md) for the LLM design. The LLM node does not exist yet.

- **Speech output** (`colman_audio_output`): `pixi run speak` starts a node that speaks
  text published on `llm_response` using [Kokoro-82M](https://huggingface.co/hexgrad/Kokoro-82M)
  (Apache 2.0). It needs no API key. Model weights download on first run, and it publishes
  `tts_speaking` while audio plays. Try it with:

  ```
  ros2 topic pub --once llm_response std_msgs/msg/String "{data: 'Hello'}"
  ```

- **Speech input** (`colman_audio_input`): `ros2 run colman_audio_input speech_to_text` publishes
  recognized speech on `/speech_text`. It is still Azure-based and needs `AZURE_SPEECH_KEY` and
  `AZURE_SPEECH_REGION` in the environment. The dependencies for a local Parakeet replacement
  are in `pixi.toml`, but the node is not written yet.

For more detail on how the packages fit together, see
[`docs/repository_summary.md`](docs/repository_summary.md).

## Coval LEM Series Vacuum Pump

[Documentation](https://doc.coval.com/g/LEM/doc/lem_doc_coval_2022_v06_us.pdf)
[Operating Instructions](https://doc.coval.com/g/LEM/not/lem_notice_coval_v7a_2023_01.pdf)
