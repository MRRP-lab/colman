"""The camera publisher survives the ways a real OAK camera misbehaves.

Complements test_camera_publisher.py (missing, stall, reconnect). Here: a device that
fails half way through opening, frames that error out mid-read, frames whose size
differs from the configured one, bad frames, and a long flapping run. depthai is
replaced by fakes; no camera or USB is touched.
"""

import numpy as np
import pytest
import rclpy
from test_camera_publisher import (
    FakeCameraPublisher,
    FakeDevice,
    FakePacket,
    FakePipeline,
    FakeQueue,
)

from colman_perception import camera_publisher


@pytest.fixture(autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class SizedPacket(FakePacket):
    def __init__(self, width, height, fx=900.0):
        self.width, self.height, self.fx = width, height, fx

    def getCvFrame(self):
        return np.zeros((self.height, self.width, 3), np.uint8)

    def getTransformation(self):
        fx, cx, cy = self.fx, self.width / 2, self.height / 2

        class Transform:
            def getIntrinsicMatrix(self):
                return [[fx, 0, cx], [0, fx, cy], [0, 0, 1.0]]

        return Transform()


class BrokenPacket(FakePacket):
    def __init__(self, error):
        self.error = error

    def getCvFrame(self):
        raise self.error


class NoFramePacket(FakePacket):
    def getCvFrame(self):
        return None


def capture(node):
    sent, infos = [], []
    node.image_pub.publish = sent.append
    node.info_pub.publish = infos.append
    return sent, infos


# ------------------------------------------------------------ opening half way
class FakeDai:
    """Stands in for the depthai module: records every device it opens."""

    CameraControl = camera_publisher.dai.CameraControl
    CameraBoardSocket = camera_publisher.dai.CameraBoardSocket
    node = camera_publisher.dai.node

    def __init__(self, fail_pipeline=True):
        self.devices = []
        self.fail_pipeline = fail_pipeline

    def Device(self, info):
        device = FakeDevice()
        self.devices.append(device)
        return device

    def Pipeline(self, device):
        if self.fail_pipeline:
            raise RuntimeError("pipeline build failed")
        return FakePipeline()


def test_a_device_that_fails_while_opening_is_closed(monkeypatch):
    fake = FakeDai()
    monkeypatch.setattr(camera_publisher, "dai", fake)
    monkeypatch.setattr(camera_publisher, "device_info", lambda name: name)
    node = camera_publisher.CameraPublisher()
    assert node.device is None and len(fake.devices) == 1
    # Left open, the camera stays busy and every retry fails with "already in use".
    assert fake.devices[0].closed


# ------------------------------------------------------------ errors while reading
def test_an_error_reading_a_frame_does_not_kill_the_node():
    node = FakeCameraPublisher([[BrokenPacket(RuntimeError("usb"))], [FakePacket()]])
    sent, _ = capture(node)
    node.timer_callback()  # must not raise
    node.reconnect_callback()
    node.timer_callback()
    assert len(sent) == 1


def test_an_empty_frame_is_skipped():
    node = FakeCameraPublisher([[NoFramePacket(), FakePacket()]])
    sent, _ = capture(node)
    node.timer_callback()
    node.timer_callback()
    assert len(sent) == 1


# ------------------------------------------------------------ resolution change
def test_camera_info_follows_the_frame_size():
    node = FakeCameraPublisher([[SizedPacket(1920, 1080, fx=1460.0)]])
    sent, infos = capture(node)
    node.timer_callback()
    assert (sent[0].width, sent[0].height) == (1920, 1080)
    assert (infos[0].width, infos[0].height) == (1920, 1080)
    assert infos[0].k[0] == pytest.approx(1460.0)
    assert infos[0].k[2] == pytest.approx(960.0)


def test_resolution_change_on_reconnect_updates_camera_info():
    node = FakeCameraPublisher(
        [[SizedPacket(3840, 2160, fx=2920.0)], [SizedPacket(1920, 1080, fx=1460.0)]]
    )
    sent, infos = capture(node)
    node.timer_callback()
    node.opened[0].closed = True
    node.timer_callback()
    node.reconnect_callback()
    node.timer_callback()
    assert (infos[-1].width, infos[-1].height) == (1920, 1080)
    assert infos[-1].k[0] == pytest.approx(1460.0)
    assert (sent[-1].width, sent[-1].height) == (1920, 1080)


def test_resolution_change_without_a_reconnect_updates_camera_info():
    node = FakeCameraPublisher(
        [[SizedPacket(3840, 2160, fx=2920.0), SizedPacket(1920, 1080, fx=1460.0)]]
    )
    _, infos = capture(node)
    node.timer_callback()
    node.timer_callback()
    assert (infos[-1].width, infos[-1].height) == (1920, 1080)
    assert infos[-1].k[0] == pytest.approx(1460.0)


# ------------------------------------------------------------ flapping
def test_repeated_dropouts_always_recover_and_never_leak(monkeypatch):
    monkeypatch.setattr(camera_publisher, "STALL_S", 0.0)
    script = []
    for _ in range(10):
        script += [RuntimeError("busy"), [FakePacket()]]
    node = FakeCameraPublisher(script)
    sent, _ = capture(node)
    for _ in range(40):
        node.reconnect_callback()
        node.timer_callback()  # delivers the frame
        node.timer_callback()  # nothing more: stalled at once, so it drops
    assert len(sent) == 10
    assert all(d.closed for d in node.opened[:-1])


def test_refocus_when_the_control_queue_fails():
    node = FakeCameraPublisher([[]])

    class Dead(FakeQueue):
        def send(self, control):
            raise RuntimeError("device closed")

    node.control_queue = Dead([])
    response = node.on_refocus(None, type("R", (), {})())
    assert response.success is False and "refocus failed" in response.message


def test_shutdown_with_no_camera_is_clean():
    node = FakeCameraPublisher([RuntimeError("none")])
    node.disconnect()
    node.disconnect()  # twice is fine
    assert node.device is None
