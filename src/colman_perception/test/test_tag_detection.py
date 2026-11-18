"""AprilTag detection regression on synthetic frames.

The stack runs apriltag_ros (the C apriltag library, tag36h11, max_hamming 0, no
decimation, edge refinement; see config/apriltag*.yaml). The same library is available
to Python as `apriltag`, so these tests run it on rendered scenes and check the ids and
the pose recovered by planar PnP, as apriltag_ros does with pose_estimation_method
"pnp". The apriltag_ros node itself needs a running ROS graph and is not started here.
Offline, GPU-free; the 4K cases take a fraction of a second each.
"""

import numpy as np
import pytest
import yaml

cv2 = pytest.importorskip("cv2")
apriltag = pytest.importorskip("apriltag")

from tag_scene import (  # noqa: E402
    K_4K,
    K_1080P,
    SIZE_4K,
    SIZE_1080P,
    TAG_SIZE_M,
    angle_deg,
    render,
    rotation,
    solve_pose,
)

from colman_perception import camera_publisher  # noqa: E402

# 1080p: tag ~30 px wide at the overhead camera's 1.83 m; 4K doubles that.
OVERHEAD_Z = 1.83
LATERAL_TOL_M = 0.003
DEPTH_TOL_M = 0.012
ROT_TOL_DEG = 3.0


@pytest.fixture(scope="module")
def detector():
    # Same settings as config/apriltag_overhead.yaml
    return apriltag.apriltag(
        "tag36h11", threads=2, maxhamming=0, decimate=1.0, blur=0.0, refine_edges=True
    )


def detect(detector, image):
    return {d["id"]: d for d in detector.detect(image)}


def check_pose(
    detection,
    k,
    truth_r,
    truth_t,
    lateral=LATERAL_TOL_M,
    depth=DEPTH_TOL_M,
    rot_deg=ROT_TOL_DEG,
):
    rot, trans = solve_pose(detection["lb-rb-rt-lt"], k)
    error = trans - truth_t
    assert np.abs(error[:2]).max() < lateral, error
    assert abs(error[2]) < depth, error
    assert angle_deg(rot, truth_r) < rot_deg


def test_detector_settings_match_the_launch_config():
    from ament_index_python.packages import get_package_share_directory

    path = (
        get_package_share_directory("colman_perception")
        + "/config/apriltag_overhead.yaml"
    )
    params = yaml.safe_load(open(path))["apriltag_overhead"]["ros__parameters"]
    assert params["family"] == "36h11"
    assert params["max_hamming"] == 0
    assert params["detector"]["decimate"] == 1.0
    assert set(params["tag"]["sizes"]) == {TAG_SIZE_M}


@pytest.mark.parametrize(
    "k, size",
    [(K_1080P, SIZE_1080P), (K_4K, SIZE_4K)],
    ids=["1080p", "4k"],
)
@pytest.mark.parametrize("tilt", [0, 25, 45])
def test_overhead_tag_pose(detector, k, size, tilt):
    rot = rotation(tilt, tilt / 2, 20)
    trans = np.array([0.10, -0.05, OVERHEAD_Z])
    found = detect(detector, render([(3, rot, trans)], k, size))
    assert set(found) == {3}
    check_pose(found[3], k, rot, trans)


def test_wrist_range_pose_is_tight(detector):
    rot, trans = rotation(20, -15, 70), np.array([0.02, 0.01, 0.30])
    found = detect(detector, render([(5, rot, trans)], K_1080P, SIZE_1080P))
    assert set(found) == {5}
    check_pose(found[5], K_1080P, rot, trans, lateral=0.0005, depth=0.001)


@pytest.mark.parametrize("size_k", [(SIZE_1080P, K_1080P), (SIZE_4K, K_4K)])
def test_all_eight_tags_are_told_apart(detector, size_k):
    size, k = size_k
    tags, truth = [], {}
    for i in range(1, 9):
        trans = np.array(
            [-0.35 + 0.1 * ((i - 1) % 4), -0.1 + 0.2 * ((i - 1) // 4), 1.83]
        )
        rot = rotation(0, 0, 40 * i)
        tags.append((i, rot, trans))
        truth[i] = (rot, trans)
    found = detect(detector, render(tags, k, size))
    assert set(found) == set(truth)
    for i, (rot, trans) in truth.items():
        check_pose(found[i], k, rot, trans, depth=0.02, rot_deg=10.0)


# --------------------------------------------------------------- degraded images
def scene(contrast=1.0):
    rot, trans = rotation(10, 5, 30), np.array([0.05, 0.02, OVERHEAD_Z])
    return render([(2, rot, trans)], K_4K, SIZE_4K, tag_contrast=contrast), rot, trans


def test_blur_is_tolerated(detector):
    image, rot, trans = scene()
    image = cv2.GaussianBlur(image, (0, 0), 2.0)  # a poorly focused 4K frame
    found = detect(detector, image)
    assert set(found) == {2}
    check_pose(found[2], K_4K, rot, trans, lateral=0.004, depth=0.02, rot_deg=10.0)


def test_sensor_noise_is_tolerated(detector):
    image, rot, trans = scene()
    rng = np.random.default_rng(0)
    noisy = image + rng.normal(0, 10, image.shape)
    image = np.clip(noisy, 0, 255).astype(np.uint8)
    found = detect(detector, image)
    assert set(found) == {2}
    check_pose(found[2], K_4K, rot, trans, lateral=0.004, depth=0.02, rot_deg=10.0)


def test_low_contrast_is_tolerated(detector):
    image, rot, trans = scene(contrast=0.25)
    found = detect(detector, image)
    assert set(found) == {2}
    check_pose(found[2], K_4K, rot, trans, lateral=0.004, depth=0.02, rot_deg=10.0)


def test_dim_scene_with_noise_and_blur_together(detector):
    image, rot, trans = scene(contrast=0.5)
    image = cv2.GaussianBlur(image, (0, 0), 1.2)
    rng = np.random.default_rng(1)
    image = np.clip(image + rng.normal(0, 6, image.shape), 0, 255).astype(np.uint8)
    found = detect(detector, image)
    assert set(found) == {2}
    check_pose(found[2], K_4K, rot, trans, lateral=0.005, depth=0.025)


def tag_pixel_box(rot, trans, k):
    rvec, _ = cv2.Rodrigues(rot)
    half = TAG_SIZE_M / 2
    corners = np.array(
        [[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]]
    )
    pixels, _ = cv2.projectPoints(corners, rvec, trans, k, None)
    pixels = pixels.reshape(-1, 2)
    return pixels.min(0).astype(int), pixels.max(0).astype(int)


def test_occluding_the_data_bits_never_gives_a_wrong_id(detector):
    image, rot, trans = scene()
    lo, hi = tag_pixel_box(rot, trans, K_4K)
    centre = (lo + hi) // 2
    image[centre[1] - 6 : hi[1], centre[0] - 6 : hi[0]] = 90  # cover a block of cells
    found = detect(detector, image)
    assert set(found) <= {2}  # lost is fine, a different id is not


def test_covering_most_of_the_tag_loses_it(detector):
    image, rot, trans = scene()
    lo, hi = tag_pixel_box(rot, trans, K_4K)
    image[lo[1] - 5 : hi[1] + 5, lo[0] + (hi[0] - lo[0]) // 3 : hi[0] + 5] = 200
    assert detect(detector, image) == {}


def test_a_tag_next_to_an_occluded_one_is_still_found(detector):
    rot = rotation(0, 0, 0)
    a, b = np.array([-0.1, 0.0, OVERHEAD_Z]), np.array([0.1, 0.0, OVERHEAD_Z])
    image = render([(1, rot, a), (4, rot, b)], K_4K, SIZE_4K)
    lo, hi = tag_pixel_box(rot, a, K_4K)
    image[lo[1] - 4 : hi[1] + 4, lo[0] - 4 : hi[0] + 4] = 100  # a hand over tag 1
    found = detect(detector, image)
    assert set(found) == {4}
    check_pose(found[4], K_4K, rot, b, depth=0.02, rot_deg=10.0)


# --------------------------------------------------------------- no false detections
@pytest.mark.parametrize("seed", range(3))
def test_noise_and_clutter_give_no_tags(detector, seed):
    rng = np.random.default_rng(seed)
    image = rng.integers(0, 256, (1080, 1920), dtype=np.uint8)
    assert detect(detector, image) == {}
    image = np.full((1080, 1920), 200, np.uint8)
    cv2.rectangle(image, (100, 100), (400, 400), 0, 8)
    cv2.circle(image, (900, 500), 200, 30, -1)
    assert detect(detector, image) == {}


def test_a_tag_outside_the_stack_family_is_ignored(detector):
    """A tag of another family (aruco 4x4) must not be reported as a 36h11 tag."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    cells = cv2.aruco.generateImageMarker(dictionary, 7, 6, borderBits=1)
    big = cv2.resize(cells, (240, 240), interpolation=cv2.INTER_NEAREST)
    image = np.full((600, 600), 255, np.uint8)
    image[180:420, 180:420] = big
    assert detect(detector, image) == {}


def test_overhead_stream_size_matches_the_test_intrinsics():
    width, height = camera_publisher.CAMERAS["overhead"][2]
    assert (width, height) == SIZE_4K
    assert K_4K[0, 2] == pytest.approx(width / 2, abs=5)
