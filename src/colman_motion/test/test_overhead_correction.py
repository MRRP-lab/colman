from types import SimpleNamespace

import pytest
from colman_motion.overhead_correction import (
    CAMERA_XYZ,
    correct_overhead,
    corrected_z,
)


def transform(x, y, z):
    return SimpleNamespace(translation=SimpleNamespace(x=x, y=y, z=z), rotation="r")


def test_known_heights_are_close_to_the_true_heights():
    # what the overhead camera read for single cubes and the two stacks at 4K
    for measured, true in [(0.038, 0.05), (0.095, 0.10), (0.136, 0.15)]:
        assert corrected_z(measured) == pytest.approx(true, abs=0.007)


def test_directly_below_the_camera_only_z_changes():
    cam_x, cam_y, _ = CAMERA_XYZ
    out = correct_overhead(transform(cam_x, cam_y, 0.04)).translation
    assert (out.x, out.y) == pytest.approx((cam_x, cam_y))
    assert out.z == pytest.approx(corrected_z(0.04))


def test_off_axis_x_y_move_along_the_line_of_sight_toward_the_camera_axis():
    cam_x, cam_y, cam_z = CAMERA_XYZ
    before = transform(cam_x + 0.5, cam_y, 0.04)
    out = correct_overhead(before).translation
    # the tag is higher than measured, so it is nearer the camera's axis in x and y
    assert out.z > 0.04
    assert cam_x < out.x < cam_x + 0.5
    # the corrected point is still on the line from the camera through the measurement
    ratio_xy = (out.x - cam_x) / 0.5
    ratio_z = (cam_z - out.z) / (cam_z - 0.04)
    assert ratio_xy == pytest.approx(ratio_z)


def test_the_input_is_not_changed_and_the_rotation_is_kept():
    before = transform(0.4, 0.1, 0.04)
    out = correct_overhead(before)
    assert before.translation.z == 0.04
    assert out.rotation == "r"
