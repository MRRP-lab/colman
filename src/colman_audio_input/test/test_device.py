"""A missing or renamed microphone must fail loudly, not hang or run deaf."""

import threading

import pytest
import rclpy
import sounddevice as sd

from colman_audio_input import speech_to_text_node as stt


def failing_stream(exc):
    class Stream:
        def __init__(self, *args, **kwargs):
            raise exc

    return Stream


def test_device_index_is_passed_as_int(stt_factory):
    stt_factory("input_device:=10")
    from conftest import StubStream

    assert StubStream.opened_with["device"] == 10


def test_device_numeric_string_becomes_int(stt_factory):
    stt_factory("input_device:='10'")
    from conftest import StubStream

    assert StubStream.opened_with["device"] == 10


def test_device_name_and_default(stt_factory):
    from conftest import StubStream

    stt_factory("input_device:=USB PnP Sound Device")
    assert StubStream.opened_with["device"] == "USB PnP Sound Device"


def test_default_device_is_none(stt_factory):
    from conftest import StubStream

    stt_factory()
    assert StubStream.opened_with["device"] is None


@pytest.mark.parametrize(
    "exc",
    [
        ValueError("No input device matching 'USB Mic'"),
        sd.PortAudioError("Error querying device 10"),
    ],
)
def test_missing_device_raises_clear_error_and_stops_threads(
    stt_factory, monkeypatch, exc
):
    monkeypatch.setattr(stt.sd, "InputStream", failing_stream(exc))
    before = threading.active_count()
    with pytest.raises(RuntimeError, match="Could not open the microphone"):
        stt_factory("input_device:=10")
    # The VAD/ASR worker threads must not be left running.
    assert threading.active_count() <= before


def test_main_exits_nonzero_when_mic_missing(stt_factory, monkeypatch):
    """launch and `ros2 run` must see the failure, not a clean exit 0."""
    monkeypatch.setattr(stt.sd, "InputStream", failing_stream(ValueError("nope")))
    real_init = rclpy.init
    monkeypatch.setattr(
        rclpy,
        "init",
        lambda *a, **k: real_init(args=["--ros-args", "-p", "device:=cpu"]),
    )
    with pytest.raises(SystemExit) as info:
        stt.main()
    assert info.value.code not in (0, None)
