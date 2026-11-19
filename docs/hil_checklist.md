# Hardware-in-the-loop checklist

A repeatable reliability run for the real UR3e. It is not part of CI. `scripts/hil_check.py` drives the running stack over its normal ROS interfaces and records pass/fail per step, so runs can be compared over time.

## Setup

1. Cube stacks laid out as the demo expects, tags `tag_1` (place target) and `tag_2` (cube to pick) on the table and visible to the overhead camera with the arm at home.
2. Start the stack (`pixi run stack`, or `pixi run motion`, `pixi run driver`, `pixi run overhead`/`detect`, `pixi run speak` separately). The driver supervisor must be running for the recovery step.
3. Read the plan first. It sends nothing:

```bash
pixi run hil-check --dry-run --cycles 3
```

4. Run for real:

```bash
pixi run hil-check --i-am-at-the-robot --cycles 3
```

Useful flags: `--skip STEP` (repeatable, e.g. `--skip driver_recovery`), `--tags tag_1,tag_2,tag_5`, `--pick-tag`, `--place-tag`, `--stop-delay SECONDS`, `--stop-target POSE`, `--speak` (announce progress on `llm_response`), `--results-dir`.

## Safety

- Stay at the robot with the pendant e-stop in hand. Without `--i-am-at-the-robot` the script refuses to run (except `--dry-run`).
- Clear the workspace. The stop test deliberately halts the arm mid-move.
- The first failed step ends the run and calls `/stop`. Ctrl+C also calls `/stop`. After an abort, check the arm and call `/resume` yourself before running anything else.
- The driver step asks you to kill the driver by hand; type `skip` at the prompt to skip it.
- Tests for the tool itself run offline: `pixi run test-hil`.

## Steps

| Step | Passes when |
| --- | --- |
| preflight | `/move_to`, `/pick`, `/place` servers and `/stop`, `/resume`, `/grasp`, `/release` services exist; something listens on `llm_response`; `/joint_states`, `/oakd_pro/rgb/image` and `/overhead/rgb/image` publish; every `--tags` tag has a TF to `base_link`; `/release` (vacuum off) answers |
| home | `MoveTo home` succeeds |
| check_tags | expected tags are visible with the arm at home |
| pick | `Pick` of `--pick-tag` succeeds |
| place | `Place` at `--place-tag` succeeds |
| stop_mid_motion | `MoveTo` is accepted, `/stop` after `--stop-delay` ends the goal unsuccessfully, and a new goal is refused while stopped |
| resume | `/resume` succeeds and the arm moves home again |
| pick_again | pick works after a stop and resume |
| place_again | the cube is put back |
| driver_recovery | after you kill the driver: `/joint_states` return and a home move works |

## Reading results

Each run writes `hil_results/hil_<timestamp>.json` (config, summary, per-step results) and a `.csv` with the same rows. The directory is git-ignored. The table printed at the end shows cycle, step, status, seconds and message; cycle 0 is preflight.

Track the pass rate over time:

```bash
python -c "import json,glob;[print(f, json.load(open(f))['summary']['pass_rate']) for f in sorted(glob.glob('hil_results/*.json'))]"
```

Compare step durations between runs: a step that slows down gradually (planning retries, slow driver reconnect) usually fails soon after.

## Known failure signatures

- `tag_N not visible` on check_tags or pick: overhead camera lost the tag, lighting, a cube moved, or the arm is not clear of the camera view at home.
- `goal rejected` right after a stop: `/resume` was not called, or the server is still busy with an earlier goal.
- `move completed despite /stop`: the stop came after the move finished (raise the target distance or lower `--stop-delay`), or `/stop` is not halting execution. Treat the second as a real bug.
- `goal still running after /stop`: halting did not end the goal.
- `no /joint_states` after driver recovery: the supervisor did not bring the driver back; check `pixi run driver` logs.
- `Not holding anything` on place: vacuum did not grip. Check air supply, cube position, the `/grasp` path.
- Timeouts on pick or place: planning retries; look at the motion server log for "Execution" or IK errors.
- Preflight `nobody listens on llm_response`: TTS node (`pixi run speak`) is down. Skip nothing silently; start it or accept the failure.
