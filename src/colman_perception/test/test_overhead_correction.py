from pathlib import Path

import pytest
import yaml
from colman_perception.overhead_correction import (
    CAMERA_XYZ,
    correct_position,
    corrected_z,
)

# Raw overhead readings (x, y, z) of tags at exactly known heights: a cube top is
# -0.005 + 0.05 n, the table surface being at z = -0.005 in base_link.
MEASURED = [
    ((0.2291, -0.0103, 0.03559), 1),  # eight singles spread over the table
    ((0.4614, -0.0132, 0.04549), 1),  # the one that read almost the true height
    ((0.3482, -0.2905, 0.03331), 1),
    ((0.3430, 0.2659, 0.02568), 1),  # the one that read lowest
    ((0.6064, 0.5402, 0.03412), 1),
    ((0.1117, -0.5404, 0.02776), 1),
    ((0.6034, -0.5397, 0.03966), 1),
    ((0.1163, 0.5415, 0.02969), 1),
    ((0.1535, -0.4860, 0.1365), 3),
    ((0.5345, 0.4640, 0.1327), 3),
    ((0.4335, -0.0355, 0.1777), 4),  # a stack grown from 4 to 8 at one spot
    ((0.4304, -0.0361, 0.3776), 8),
]


def exact_top(n):
    return -0.005 + 0.05 * n


def test_known_heights_are_close_to_the_exact_heights_everywhere_on_the_table():
    for position, n in MEASURED:
        z = correct_position(position)[2]
        # Good to about 1.5 cm: enough to tell which level a cube is on, since the
        # levels are 5 cm apart, and not enough to send the arm to.
        assert z == pytest.approx(exact_top(n), abs=0.02)


def test_the_correction_alone_never_puts_a_cube_on_the_wrong_level():
    for position, n in MEASURED:
        z = correct_position(position)[2]
        assert round((z + 0.005) / 0.05) == n


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
