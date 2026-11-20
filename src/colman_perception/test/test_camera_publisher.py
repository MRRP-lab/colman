"""The camera publisher recovers from a camera that is missing, drops off, or stalls."""

import datetime
import threading
import time

import cv2
import numpy as np
import pytest
import rclpy
from colman_perception.camera_publisher import CameraPublisher
from rclpy.parameter import Parameter

from colman_perception import camera_publisher


class FakePacket:
    def getCvFrame(self):
        return np.zeros((720, 1280, 3), np.uint8)

    def getLensPosition(self):
        return 120

    def getExposureTime(self):
        return datetime.timedelta(microseconds=8000)

    def getSensitivity(self):
        return 200

    def getTransformation(self):
        class Transform:
            def getIntrinsicMatrix(self):
                return [[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]]

        return Transform()


class FakeJpegPacket(FakePacket):
    """A frame as the camera's encoder sends it: JPEG bytes and no getCvFrame."""

    def __init__(self, value=90):
        ok, encoded = cv2.imencode(".jpg", np.full((72, 128, 3), value, np.uint8))
        self.data = encoded

    def getData(self):
        return self.data


class FakeQueue:
    def __init__(self, packets):
        self.packets = packets

    def tryGet(self):
        return self.packets.pop(0) if self.packets else None

    def send(self, control):
        pass


class FakeDevice:
    def __init__(self):
        self.closed = False

    def isClosed(self):
        return self.closed

    def close(self):
        self.closed = True


class FakePipeline:
    def stop(self):
        pass


class FakeCameraPublisher(CameraPublisher):
    """Opens fake cameras. `script` lists what each open does: an error, or packets."""

    def __init__(self, script, camera="wrist", **parameters):
        self.script = list(script)
        self.opened = []
        overrides = [Parameter("camera", value=camera)]
        overrides += [Parameter(k, value=v) for k, v in parameters.items()]
        super().__init__(parameter_overrides=overrides)

    def open_camera(self):
        step = self.script.pop(0) if self.script else []
        if isinstance(step, Exception):
            raise step
        device = FakeDevice()
        self.opened.append(device)
        return device, FakePipeline(), FakeQueue(list(step)), FakeQueue([])


@pytest.fixture(autouse=True)
def ros(monkeypatch):
    # These tests check the camera code, not the transport. The large-image Fast DDS
    # profile (pixi.toml) reserves a 512 MB shared-memory segment for every node, and
    # a CI runner cannot give one to a node per test: the node then fails to start
    # with "failed to create participant". It must be dropped before the first node.
    monkeypatch.delenv("FASTRTPS_DEFAULT_PROFILES_FILE", raising=False)
    rclpy.init()
    yield
    rclpy.shutdown()


def test_retries_until_the_camera_appears():
    node = FakeCameraPublisher([RuntimeError("busy"), RuntimeError("busy"), []])
    assert node.device is None
    node.reconnect_callback()
    assert node.device is None
    node.reconnect_callback()
    assert node.device is not None and len(node.opened) == 1


def test_does_not_reopen_a_working_camera():
    node = FakeCameraPublisher([[], []])
    first = node.device
    node.reconnect_callback()
    assert node.device is first and len(node.opened) == 1


def test_reconnects_when_the_device_closes():
    node = FakeCameraPublisher([[], []])
    node.opened[0].closed = True
    node.timer_callback()
    assert node.device is None
    node.reconnect_callback()
    assert node.device is not None and len(node.opened) == 2


def test_reconnects_when_frames_stop(monkeypatch):
    monkeypatch.setattr(camera_publisher, "STALL_S", 0.05)
    node = FakeCameraPublisher([[], []])
    node.timer_callback()
    assert node.device is not None  # no packet yet, but not stalled for long enough
    time.sleep(0.1)
    node.timer_callback()
    assert node.device is None
    assert node.opened[0].closed
    node.reconnect_callback()
    assert len(node.opened) == 2


def test_publishes_frames_and_keeps_going_after_a_reconnect():
    node = FakeCameraPublisher([[FakePacket()], [FakePacket()]])
    sent = []
    node.image_pub.publish = sent.append
    node.timer_callback()
    assert len(sent) == 1 and sent[0].header.frame_id == node.frame_id
    node.opened[0].closed = True
    node.timer_callback()
    node.reconnect_callback()
    node.timer_callback()
    assert len(sent) == 2


def test_refocus_says_so_when_the_camera_is_gone():
    node = FakeCameraPublisher([RuntimeError("busy")])
    response = node.on_refocus(None, type("R", (), {})())
    assert response.success is False and "not connected" in response.message


def test_refocus_works_when_connected():
    node = FakeCameraPublisher([[]])
    response = node.on_refocus(None, type("R", (), {})())
    assert response.success is True


def wait_until(condition, seconds=5.0):
    end = time.time() + seconds
    while time.time() < end:
        if condition():
            return True
        time.sleep(0.01)
    return condition()


def published(node):
    """Collect what the node publishes, as (header stamp, first pixel value)."""
    out = []
    node.image_pub.publish = lambda m: out.append((m.header.stamp, m.data[0]))
    return out


def test_wrist_camera_defaults_to_raw_1080p_at_60_fps():
    node = FakeCameraPublisher([[]])
    assert camera_publisher.CAMERAS["wrist"].size == (1920, 1080)
    assert (node.info_msg.width, node.info_msg.height) == (1920, 1080)
    assert (node.fps, node.jpeg_quality, node.pool) == (60.0, 0, None)


def test_overhead_camera_defaults_to_4k_at_30_fps_with_mjpeg_90_on_three_threads():
    node = FakeCameraPublisher([[]], camera="overhead")
    assert (node.info_msg.width, node.info_msg.height) == (3840, 2160)
    assert (node.fps, node.jpeg_quality, node.decode_threads) == (30.0, 90, 3)
    assert node.pool is not None


def test_settings_can_be_overridden_by_parameters():
    node = FakeCameraPublisher(
        [[]], camera="overhead", fps=20.0, jpeg_quality=0, decode_threads=2
    )
    assert (node.fps, node.jpeg_quality, node.pool) == (20.0, 0, None)


@pytest.mark.parametrize(
    "parameters",
    [{"fps": 0.0}, {"fps": -5.0}, {"jpeg_quality": 101}, {"jpeg_quality": -1}]
    + [{"decode_threads": 0}],
)
def test_bad_settings_are_refused(parameters):
    with pytest.raises(ValueError):
        FakeCameraPublisher([[]], **parameters)


def test_every_waiting_frame_is_published_in_one_poll():
    node = FakeCameraPublisher([[FakePacket() for _ in range(5)]])
    out = published(node)
    node.timer_callback()
    assert len(out) == 5  # a 60 fps camera has more than one frame per poll


def test_compressed_frames_are_decoded_and_published():
    node = FakeCameraPublisher(
        [[FakeJpegPacket(), FakeJpegPacket()]], camera="overhead"
    )
    out = published(node)
    node.timer_callback()
    assert wait_until(lambda: (node.publish_ready() or True) and len(out) == 2)
    assert node.pending == type(node.pending)()


def test_frames_are_published_in_arrival_order_even_if_decoded_out_of_order(
    monkeypatch,
):
    release_first = threading.Event()
    real_decode = camera_publisher.decode_jpeg

    def slow_first(data):
        frame = real_decode(data)
        if frame[0, 0, 0] == 10:  # the first frame waits for the second
            release_first.wait(5.0)
        return frame

    monkeypatch.setattr(camera_publisher, "decode_jpeg", slow_first)
    node = FakeCameraPublisher(
        [[FakeJpegPacket(10), FakeJpegPacket(200)]], camera="overhead"
    )
    out = published(node)
    node.timer_callback()
    time.sleep(0.3)  # the second frame is decoded by now, the first is not
    node.publish_ready()
    assert out == []  # nothing goes out ahead of the older frame
    release_first.set()
    assert wait_until(lambda: (node.publish_ready() or True) and len(out) == 2)
    assert [value for _, value in out] == [10, 200]


def test_a_frame_that_will_not_decode_is_dropped_and_the_stream_goes_on():
    bad = FakeJpegPacket()
    bad.data = np.frombuffer(b"not a jpeg", np.uint8)
    node = FakeCameraPublisher([[bad, FakeJpegPacket()]], camera="overhead")
    out = published(node)
    node.timer_callback()
    assert wait_until(lambda: (node.publish_ready() or True) and len(out) == 1)
    assert node.device is not None


def test_frames_are_dropped_when_decoding_cannot_keep_up(monkeypatch):
    gate = threading.Event()
    monkeypatch.setattr(
        camera_publisher, "decode_jpeg", lambda data: gate.wait(5.0) and None
    )
    n = camera_publisher.MAX_IN_FLIGHT + 5
    node = FakeCameraPublisher(
        [[FakeJpegPacket() for _ in range(n)]], camera="overhead"
    )
    while node.queue.packets:
        node.timer_callback()
    assert len(node.pending) == camera_publisher.MAX_IN_FLIGHT
    gate.set()


class DyingQueue(FakeQueue):
    """A queue that raises a non-RuntimeError, as an unplugged camera may."""

    def tryGet(self):
        raise ValueError("X_LINK_ERROR")


def test_reconnects_when_an_unplugged_camera_raises_an_unexpected_error():
    node = FakeCameraPublisher([[], []])
    node.queue = DyingQueue([])
    node.timer_callback()  # must not raise
    assert node.device is None
    node.reconnect_callback()
    assert node.device is not None and len(node.opened) == 2


def test_keeps_retrying_when_opening_raises_an_unexpected_error():
    node = FakeCameraPublisher([ValueError("X_LINK_ERROR"), []])
    assert node.device is None  # the first open failed and the node is still running
    node.reconnect_callback()
    assert node.device is not None
