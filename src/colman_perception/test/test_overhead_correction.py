from pathlib import Path

import pytest
import yaml
from colman_perception.overhead_correction import (
    CAMERA_XYZ,
    correct_position,
    corrected_z,
)


def test_known_heights_are_close_to_the_true_heights():
    # raw overhead heights read for a single cube and stacks of 2, 4 and 8 cubes
    for measured, true in [
        (0.0242, 0.05),
        (0.0724, 0.10),
        (0.1777, 0.20),
        (0.3776, 0.40),
    ]:
        assert corrected_z(measured) == pytest.approx(true, abs=0.008)


def test_directly_below_the_camera_only_z_changes():
    cam_x, cam_y, _ = CAMERA_XYZ
    x, y, z = correct_position((cam_x, cam_y, 0.04))
    assert (x, y) == pytest.approx((cam_x, cam_y))
    assert z == pytest.approx(corrected_z(0.04))


def test_off_axis_x_y_move_along_the_line_of_sight():
    cam_x, cam_y, cam_z = CAMERA_XYZ
    x, y, z = correct_position((cam_x + 0.5, cam_y, 0.04))
    assert z > 0.04
    assert cam_x < x < cam_x + 0.5
    assert (x - cam_x) / 0.5 == pytest.approx((cam_z - z) / (cam_z - 0.04))


def test_the_camera_position_matches_the_calibration_file():
    config = Path(__file__).parents[1] / "colman_perception" / "config"
    pose = yaml.safe_load(
        (config / "overhead_camera_pose_in_base_link.yaml").read_text()
    )
    x, y, z = (float(v) for v in pose["static_transform_args"][:3])
    assert CAMERA_XYZ == pytest.approx((x, y, z), abs=1e-5)
