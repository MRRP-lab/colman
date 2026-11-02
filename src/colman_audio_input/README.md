# colman_audio_input

Speech-to-text ROS 2 node using NVIDIA Parakeet. Publishes recognized text as
`std_msgs/String` on `speech_text`.

All commands run from the workspace root (`~/school/colman/colman`) inside the
pixi environment.

## Build

```bash
pixi run build
```

## Run the node

```bash
pixi run ros2 run colman_audio_input speech_to_text
```

Optional parameters: `topic` (default `speech_text`), `mute_topic`
(default `tts_speaking`, mutes the mic while the robot speaks), and
`input_device` (sounddevice index or name; empty uses the default mic).

```bash
pixi run ros2 run colman_audio_input speech_to_text --ros-args \
    -p topic:=speech_text -p input_device:="'2'"
```

The inner quotes force ROS to treat the index as a string (the parameter is
declared as a string). A device name works too: `-p input_device:="'USB Mic'"`.
List available devices with:

```bash
pixi run python -m sounddevice
```

## View output live (second terminal)

```bash
pixi run ros2 topic echo /speech_text
```

## WAV file test

Feeds a WAV file (16-bit PCM, 16 kHz) through the node instead of the
microphone. No audio hardware needed.

```bash
pixi run python src/colman_audio_input/test/wav_file_check.py path/to/sample.wav \
    --expect "expected phrase" --device cuda
```

- `--expect` can be repeated; exits 0 if every phrase appears, 1 otherwise.
- `--device` is `auto`, `cuda`, or `cpu`.
- The first run downloads the model.
