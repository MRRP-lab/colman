"""The checked-in camera pose calibration files are well formed and physically sane.

The overhead camera's pose is in colman_perception/config/ (the file the stack uses)
and a copy in camera_pose_calibration/results/. The wrist camera's mount is in
camera_pose_calibration/results/ and is copied into colman.urdf.xacro. No hardware is
needed.
"""

from pathlib import Path

import numpy as np
import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
CONFIG = Path(__file__).resolve().parents[1] / "colman_perception" / "config"
POSE = "overhead_camera_pose_in_base_link.yaml"
RESULTS = REPO / "camera_pose_calibration" / "results"
FILES = [CONFIG / POSE, RESULTS / POSE]
XACRO = REPO / "src" / "colman_description" / "colman_description" / "urdf"

# The camera hangs ~1.83 m above the table, near the robot base.
MAX_TRANSLATION_M = 3.0
Z_RANGE_M = (1.0, 2.5)


def load(path):
    return yaml.safe_load(Path(path).read_text())


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def angle_between(r1, r2):
    c = (np.trace(r1.T @ r2) - 1) / 2
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def test_calibration_files_exist():
    for path in FILES + [RESULTS / "wrist_camera_mount_on_tool_flange.yaml"]:
        assert path.is_file(), f"missing {path.relative_to(REPO)}"


@pytest.fixture(params=FILES, ids=lambda p: f"{p.parent.parent.name}/{p.name}")
def calib(request):
    return load(request.param)


def test_has_the_keys_the_launch_needs(calib):
    for key in (
        "parent_frame",
        "child_frame",
        "rotation_matrix",
        "quaternion_xyzw",
        "translation_m",
        "static_transform_args",
    ):
        assert key in calib, key
    assert calib["parent_frame"] == "base_link"
    assert calib["child_frame"] == "overhead_camera_optical_frame"


def test_rotation_is_a_proper_rotation(calib):
    r = np.array(calib["rotation_matrix"], dtype=float)
    assert r.shape == (3, 3)
    assert np.all(np.isfinite(r))
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-6)
    assert np.linalg.det(r) == pytest.approx(1.0, abs=1e-6)


def test_quaternion_is_unit_and_matches_the_rotation(calib):
    q = np.array(calib["quaternion_xyzw"], dtype=float)
    assert q.shape == (4,) and np.all(np.isfinite(q))
    assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)
    r = np.array(calib["rotation_matrix"], dtype=float)
    assert angle_between(quat_to_matrix(q), r) < 0.01


def test_translation_is_plausible(calib):
    t = np.array(calib["translation_m"], dtype=float)
    assert t.shape == (3,) and np.all(np.isfinite(t))
    assert np.linalg.norm(t) < MAX_TRANSLATION_M
    assert Z_RANGE_M[0] < t[2] < Z_RANGE_M[1]


def test_camera_looks_down_at_the_table(calib):
    # The optical axis (+z of the camera) must point down in base_link.
    r = np.array(calib["rotation_matrix"], dtype=float)
    assert r[2, 2] < -0.9


def test_static_transform_args_are_well_formed(calib):
    args = calib["static_transform_args"]
    assert len(args) == 9
    values = [float(a) for a in args[:7]]  # raises if not numeric
    assert np.all(np.isfinite(values))
    assert np.linalg.norm(values[3:7]) == pytest.approx(1.0, abs=1e-3)
    assert args[7:] == [calib["parent_frame"], calib["child_frame"]]


def test_static_transform_args_are_close_to_the_fit(calib):
    args = calib["static_transform_args"]
    t = np.array(calib["translation_m"], dtype=float)
    assert np.linalg.norm(np.array(args[:3], dtype=float) - t) < 0.03
    r = quat_to_matrix([float(a) for a in args[3:7]])
    assert angle_between(r, np.array(calib["rotation_matrix"], dtype=float)) < 2.0


def test_intrinsics_are_positive_when_present(calib):
    if "intrinsics" not in calib:
        pytest.skip("this file has no intrinsics")
    i = calib["intrinsics"]
    for key in ("fx", "fy", "cx", "cy", "width", "height"):
        assert np.isfinite(i[key]) and i[key] > 0, key
    assert 0 < i["cx"] < i["width"] and 0 < i["cy"] < i["height"]
    # Plausible field of view for the OAK-D Pro (tens of degrees to ~120).
    fov = 2 * np.degrees(np.arctan(i["width"] / 2 / i["fx"]))
    assert 30 < fov < 120
    assert i["fx"] == pytest.approx(i["fy"], rel=0.01)  # square pixels


def test_active_config_is_the_same_as_the_results_copy():
    active = load(CONFIG / POSE)
    kept = load(RESULTS / POSE)
    for key in ("rotation_matrix", "quaternion_xyzw", "translation_m"):
        assert np.allclose(active[key], kept[key]), key
    assert active["static_transform_args"] == kept["static_transform_args"]


def test_active_config_args_match_its_own_fit():
    active = load(CONFIG / POSE)
    args = active["static_transform_args"]
    # args are rounded to 5 decimals in the file
    assert np.allclose([float(a) for a in args[:3]], active["translation_m"], atol=1e-4)
    assert np.allclose(
        [float(a) for a in args[3:7]], active["quaternion_xyzw"], atol=1e-4
    )


# ------------------------------------------------------------------ wrist camera
@pytest.fixture
def wrist():
    return load(RESULTS / "wrist_camera_mount_on_tool_flange.yaml")


def test_the_wrist_mount_is_a_small_unit_transform_from_the_flange(wrist):
    assert wrist["parent_frame"] == "tool0"
    q = np.array(wrist["quaternion_xyzw"], dtype=float)
    assert q.shape == (4,) and np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)
    t = np.array(wrist["translation_m"], dtype=float)
    assert t.shape == (3,) and np.all(np.isfinite(t)) and np.linalg.norm(t) < 0.3


def test_the_urdf_uses_the_fitted_wrist_mount(wrist):
    import re

    text = (XACRO / "colman.urdf.xacro").read_text()
    live = re.sub(
        r"<!--.*?-->", "", text, flags=re.DOTALL
    )  # not the old, commented ones
    joints = re.findall(r"<xacro:oakd_pro_joint\s+([^>]*?)/>", live)
    assert len(joints) == 1, "expected one live oakd_pro_joint"
    values = dict(re.findall(r'(\w+)="([^"]*)"', joints[0]))
    xyz = [float(values[k]) for k in "xyz"]
    assert np.allclose(xyz, wrist["translation_m"], atol=1e-5)
