"""The camera publisher recovers from a camera that is missing, drops off, or stalls."""

import datetime
import time

import numpy as np
import pytest
import rclpy
from colman_perception.camera_publisher import CameraPublisher

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

    def __init__(self, script):
        self.script = list(script)
        self.opened = []
        super().__init__()

    def open_camera(self):
        step = self.script.pop(0) if self.script else []
        if isinstance(step, Exception):
            raise step
        device = FakeDevice()
        self.opened.append(device)
        return device, FakePipeline(), FakeQueue(list(step)), FakeQueue([])


@pytest.fixture(autouse=True)
def ros():
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


def test_overhead_camera_runs_at_4k():
    assert camera_publisher.CAMERAS["overhead"][2] == (3840, 2160)
    node = FakeCameraPublisher([[]])
    assert (node.info_msg.width, node.info_msg.height) == (1280, 720)  # wrist default


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
