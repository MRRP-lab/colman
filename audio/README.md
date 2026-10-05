# audio

> **Temporary.** This directory is a stopgap. Once the speech-input, AI-model,
> and speech-output pieces have been turned into ROS nodes (see
> `docs/architecture/architecture.dot`), delete this directory.

The source here was recovered from `.pyc` files (Python 3.12) after the
original `.py` files were deleted. The bytecode matches the originals
exactly, but **comments were not recoverable** — every `#` comment is a
best-guess reconstruction. Docstrings are original. The code appears to be
derived from DougDoug's "Babagaboosh" project.

## What it is

A set of small wrapper classes for a voice-driven AI character. Each file
wraps one external service and has a `__main__` block for manual testing.
There's no orchestrating main loop here; that script wasn't recovered.

Imports are flat (e.g. `from websockets_auth import ...`), so run things from
inside `audio/`.

## Modules

- **`azure_speech_to_text.py`** — Speech-to-text via Azure Speech.
  `SpeechToTextManager.speechtotext_from_mic_continuous(stop_key='p')` listens
  to the default mic continuously, accumulates recognized phrases, and returns
  the joined transcript when you press **P** (polled via the `keyboard` lib).
  Also has one-shot mic, one-shot file, and continuous file variants.
  Env: `AZURE_TTS_KEY`, `AZURE_TTS_REGION` (misnamed "TTS"; it's STT).

- **`openai_chat.py`** — The "brain". `OpenAiManager.chat_with_history(prompt)`
  appends to a running conversation, calls `gpt-4o-mini`, stores and returns
  the reply. If history exceeds ~8000 tokens (counted with `tiktoken`) it
  drops the oldest messages, preserving index 0 (intended to be the
  system/persona prompt). `chat()` is the stateless version.
  Env: `OPENAI_API_KEY`.

- **`eleven_labs.py`** — The "voice". Text-to-speech via ElevenLabs (model
  `eleven_flash_v2`, default voice `"Doug VO Only"`). Can save to a
  `___Msg<hash>.wav/.mp3` file and return the path, play directly, or stream.
  Uses the **pre-1.0 `elevenlabs` SDK** API (`generate`, `set_api_key`), so it
  needs an old version pinned.
  Env: `ELEVENLABS_API_KEY`.

- **`audio_player.py`** — Plays audio files with pygame. Can block for the
  file's duration (length read via `soundfile` for WAV, `mutagen` for MP3) and
  optionally delete the file afterwards. Also has an async variant.

- **`obs_websockets.py`** / **`websockets_auth.py`** — Control OBS Studio over
  its websocket API: switch scenes, toggle source visibility and filters,
  get/set text sources and transforms. Originally used to animate the
  character on stream; probably not relevant to the robot.
  `websockets_auth.py` contains a plaintext password.

## How it fits together

The typical loop these pieces were built for:

```
mic → Azure STT → text → OpenAI (with persona/history) → reply
    → ElevenLabs TTS → audio file → AudioManager plays it
                                  ↘ OBS: animate character while it talks
```

This maps onto the speech-input, AI-model, and speech-output nodes in
`docs/architecture/architecture.dot`. What's missing for the robot is the ROS
layer: wrapping these as ROS nodes, and adding tool/function calling to
`OpenAiManager` so the model can emit commands like "goto A" to the arm.
