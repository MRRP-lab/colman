import numpy as np
from colman_perception.touch_off_check import (
    compare_pose,
    refine_mark,
    similarity_fit,
    table_points,
)
from colman_perception.touch_off_record import quaternion_to_matrix, tip_position

K = np.array([[1460.0, 0, 962.0], [0, 1460.0, 547.0], [0, 0, 1]])
# Camera looking straight down from 1.83 m: optical z is base -z, x is base -y.
ROTATION = np.array([[0.0, -1, 0], [-1, 0, 0], [0, 0, -1]])
TRANSLATION = np.array([0.34, 0.05, 1.83])
POSE = {"rotation_matrix": ROTATION.tolist(), "translation_m": TRANSLATION.tolist()}


def project(points_xy, height, rotation=ROTATION, translation=TRANSLATION):
    """base_link points on z = height to pixels."""
    xyz = np.c_[points_xy, np.full(len(points_xy), height)]
    cam = (xyz - translation) @ rotation
    uv = (K @ cam.T).T
    return uv[:, :2] / uv[:, 2:]


def touched_grid(height=0.004):
    xy = np.array([[x, y] for x in (0.2, 0.3, 0.4) for y in (-0.1, 0.0, 0.1)])
    return xy, np.c_[xy, np.full(len(xy), height)]


def test_table_points_inverts_projection():
    xy, _ = touched_grid()
    pixels = project(xy, 0.004)
    assert np.allclose(table_points(pixels, K, ROTATION, TRANSLATION, 0.004), xy)


def test_correct_pose_has_no_error():
    xy, xyz = touched_grid()
    result = compare_pose(project(xy, 0.004), K, POSE, xyz)
    assert np.abs(result["error_mm"]).max() < 1e-6
    assert abs(result["scale"] - 1) < 1e-9


def test_shifted_camera_shows_as_offset_not_rms():
    xy, xyz = touched_grid()
    # The truth is a camera 12 mm further along y than the pose under test.
    true_translation = TRANSLATION + [0, 0.012, 0]
    pixels = project(xy, 0.004, translation=true_translation)
    result = compare_pose(pixels, K, POSE, xyz)
    # Pose under test is 12 mm to one side, so every point is 12 mm off in y.
    assert np.allclose(result["offset_mm"], [0, -12, 0][:2], atol=1e-6)
    assert result["rms_mm"] < 1e-6
    assert abs(result["scale"] - 1) < 1e-9


def test_wrong_height_shows_as_scale():
    xy, xyz = touched_grid()
    wrong = dict(POSE, translation_m=(TRANSLATION + [0, 0, 0.05]).tolist())
    result = compare_pose(project(xy, 0.004), K, wrong, xyz)
    # Higher camera than the truth: the projection spreads out.
    assert result["scale"] > 1.02
    assert result["fit_rms_mm"] < 1e-6


def test_similarity_fit_recovers_scale_and_yaw():
    a = np.array([[0.0, 0], [1, 0], [1, 1], [0, 2]])
    angle = np.radians(3)
    rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    b = 1.1 * a @ rot.T + [5, -2]
    scale, yaw, rms = similarity_fit(a, b)
    assert abs(scale - 1.1) < 1e-9
    assert abs(yaw - 3) < 1e-9
    assert rms < 1e-9


def test_tip_position_applies_tool_rotation():
    # Tool rotated 90 degrees about z: its x axis points along base y.
    q = (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5))
    rotation = quaternion_to_matrix(*q)
    tip = tip_position(rotation, [0.3, 0.1, 0.2], [0.1, 0.0, 0.0])
    assert np.allclose(tip, [0.3, 0.2, 0.2])


def test_refine_mark_finds_dark_dot_centre_from_an_offset_click():
    gray = np.full((100, 100), 200, np.uint8)
    yy, xx = np.mgrid[0:100, 0:100]
    gray[(xx - 40.3) ** 2 + (yy - 55.7) ** 2 <= 9] = 30
    x, y = refine_mark(gray, 43, 52)
    assert abs(x - 40.3) < 0.5 and abs(y - 55.7) < 0.5


def test_refine_mark_keeps_click_when_nothing_is_there():
    gray = np.full((50, 50), 120, np.uint8)
    assert refine_mark(gray, 20, 25) == (20.0, 25.0)
