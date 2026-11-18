"""Run the real VAD + Parakeet on the recorded WAV (no microphone).

Marked `model`: it needs the ~600 MB Parakeet ONNX model, which CI does not
cache, so it skips unless the model is already in ~/.cache/colman/asr. Run it
locally with:  pixi run pytest -m model src/colman_audio_input/test
"""

import time
from pathlib import Path

import numpy as np
import pytest
from conftest import StubStream, spin_until
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from wav_file_check import normalize, read_wav

from colman_audio_input import speech_to_text_node as stt

HERE = Path(__file__).parent
MODEL_NAME = "nemo-parakeet-tdt-0.6b-v2"
MAX_WER = 0.15
TRAILING_SILENCE_S = 2.0  # longer than silence_ms so the VAD closes the utterance

def word_error_rate(reference, hypothesis):
    ref, hyp = normalize(reference).split(), normalize(hypothesis).split()
    d = list(range(len(hyp) + 1))
    for i, r in enumerate(ref, 1):
        prev, d[0] = d[0], i
        for j, h in enumerate(hyp, 1):
            prev, d[j] = d[j], min(d[j] + 1, d[j - 1] + 1, prev + (r != h))
    return d[-1] / max(len(ref), 1)


def test_wer_helper():
    assert word_error_rate("a b c d", "a b c d") == 0
    assert word_error_rate("a b c d", "a x c d") == 0.25
    assert word_error_rate("a b", "") == 1.0
    assert word_error_rate("Move the Red cube.", "move the red cube") == 0


@pytest.fixture
def real_node(monkeypatch, ros):
    model_dir = stt.MODEL_DIR / MODEL_NAME
    if not (model_dir / "encoder-model.onnx").exists() and not any(
        model_dir.glob("*.onnx")
    ):
        pytest.skip(f"Parakeet model not cached at {model_dir} (run the node once)")
    # Never download from a test: a cache miss is a skip, not a hang.
    real_download = stt.snapshot_download
    monkeypatch.setattr(
        stt,
        "snapshot_download",
        lambda *a, **k: real_download(*a, **{**k, "local_files_only": True}),
    )
    monkeypatch.setattr(stt.sd, "InputStream", StubStream)
    ros("device:=cpu", f"model_name:={MODEL_NAME}")
    try:
        node = stt.SpeechToTextNode()
    except Exception as exc:  # cache incomplete, onnxruntime/silero missing, ...
        pytest.skip(f"could not load speech models offline: {exc}")
    yield node
    node.destroy_node()


@pytest.mark.model
def test_recording_transcript_matches_reference(real_node):
    node = real_node
    expected = (HERE / "recording_16k.txt").read_text()
    audio = read_wav(str(HERE / "recording_16k.wav"))
    audio = np.concatenate(
        [audio, np.zeros(int(TRAILING_SILENCE_S * stt.SAMPLE_RATE), np.float32)]
    )
    audio = np.pad(audio, (0, -len(audio) % stt.BLOCK_SAMPLES))

    transcripts = []
    listener = Node("wav_listener")
    listener.create_subscription(
        String,
        node.get_parameter("topic").value,
        lambda m: transcripts.append(m.data),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(listener)
    try:
        for i in range(0, len(audio), stt.BLOCK_SAMPLES):
            block = audio[i : i + stt.BLOCK_SAMPLES].reshape(-1, 1)
            node._on_audio(block, stt.BLOCK_SAMPLES, None, None)
            while node._audio_q.qsize() > 50:  # pace so no blocks are dropped
                time.sleep(0.001)
        node._audio_q.join()
        node._segment_q.join()
        spin_until(executor, lambda: bool(transcripts), timeout=5.0)
    finally:
        listener.destroy_node()

    assert transcripts, "no transcript was published"
    got = " ".join(transcripts)
    wer = word_error_rate(expected, got)
    assert wer <= MAX_WER, f"WER {wer:.2f} > {MAX_WER}: {got!r} vs {expected!r}"
