import numpy as np
import pytest
from colman_motion.diff_ik import (
    DEFAULT_LOWER,
    DEFAULT_UPPER,
    damping_squared,
    dls_joint_velocity,
    jacobian,
    limit_joint_velocity,
    manipulability,
    tool_pose,
)

HOME = np.array([0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0])
BENT = np.array([0.3, -1.0, 1.5, -1.5, -1.2, 0.2])
CAMERA = (0.0, -0.1, 0.05)


def random_pose(rng):
    return rng.uniform(DEFAULT_LOWER, DEFAULT_UPPER)


def random_twist(rng):
    return np.concatenate([rng.normal(size=3) * 0.1, rng.normal(size=3) * 0.3])


def rotation_vector(rot):
    """Small-angle rotation vector of a rotation matrix."""
    return 0.5 * np.array(
        [rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]
    )


def test_home_pose_matches_ros_base_link():
    # The mock arm's TF reported tool0 at (0.001, 0.224, 0.694) for pan 0.
    pos, rot = tool_pose(HOME)
    assert pos == pytest.approx([0.0, 0.224, 0.694], abs=2e-3)
    assert np.allclose(rot @ rot.T, np.eye(3))
    assert np.linalg.det(rot) == pytest.approx(1.0)
    # Tool x along base x, tool y up, tool z (approach) along -y.
    assert rot == pytest.approx(
        np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]), abs=1e-9
    )


def test_pan_by_pi_mirrors_the_home_pose():
    q = HOME.copy()
    q[0] = -np.pi
    pos, _ = tool_pose(q)
    assert pos == pytest.approx([0.0, -0.224, 0.694], abs=2e-3)


def test_bent_pose_is_consistent_with_link_lengths():
    pos, _ = tool_pose(BENT)
    # Reach from the shoulder can never exceed the summed link offsets.
    assert np.linalg.norm(pos - [0, 0, 0.15185]) < 0.24355 + 0.2132 + 0.13105 + 0.2
    # Pan is a rotation about the base z axis: it keeps height and radius.
    q = BENT.copy()
    q[0] += 0.7
    moved, _ = tool_pose(q)
    assert moved[2] == pytest.approx(pos[2])
    assert np.hypot(*moved[:2]) == pytest.approx(np.hypot(*pos[:2]))


@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0), CAMERA])
def test_jacobian_matches_finite_difference(offset):
    rng = np.random.default_rng(3)
    eps = 1e-6
    offset = np.array(offset)
    for _ in range(10):
        q = random_pose(rng)
        jac = jacobian(q, offset)
        numeric = np.zeros((6, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = eps
            p1, r1 = tool_pose(q + dq)
            p0, r0 = tool_pose(q - dq)
            numeric[:3, i] = (p1 + r1 @ offset - p0 - r0 @ offset) / (2 * eps)
            numeric[3:, i] = rotation_vector(r1 @ r0.T) / (2 * eps)
        assert jac == pytest.approx(numeric, abs=1e-6)


def test_ordinary_poses_are_undamped():
    assert manipulability(jacobian(BENT)) > 0.005
    assert damping_squared(manipulability(jacobian(BENT))) == 0.0


@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0), CAMERA])
def test_tracks_the_twist_away_from_singularities(offset):
    rng = np.random.default_rng(4)
    checked = 0
    while checked < 200:
        q = random_pose(rng)
        if manipulability(jacobian(q, offset)) < 0.008:
            continue
        twist = random_twist(rng)
        q_dot = dls_joint_velocity(q, twist, offset, lower=None, upper=None)
        assert jacobian(q, offset) @ q_dot == pytest.approx(twist, abs=1e-3)
        checked += 1


def singular_q(kind):
    q = np.array([0.4, -1.2, 1.3, -1.0, 1.0, 0.2])
    if kind == "elbow":
        q[2] = 0.0
    elif kind == "wrist_2":
        q[4] = 0.0
    elif kind == "home":
        q = HOME.copy()
    return q


@pytest.mark.parametrize("kind", ["elbow", "wrist_2", "home"])
def test_singular_poses_stay_bounded_in_every_direction(kind):
    q = singular_q(kind)
    assert manipulability(jacobian(q)) < 1e-9
    rng = np.random.default_rng(5)
    for _ in range(200):
        lin, ang = rng.normal(size=3), rng.normal(size=3)
        twist = np.concatenate(
            [0.1 * lin / np.linalg.norm(lin), 0.3 * ang / np.linalg.norm(ang)]
        )
        q_dot = dls_joint_velocity(q, twist, lower=None, upper=None)
        assert np.isfinite(q_dot).all()
        assert np.abs(q_dot).max() < 5.0


def test_elbow_straight_tangential_is_achieved_and_radial_is_not_blown_up():
    q = singular_q("elbow")
    jac = jacobian(q)
    u, s, _ = np.linalg.svd(jac)
    assert s[-1] < 1e-9
    # The unachievable direction: radial/angular combination of the lost mode.
    radial = u[:, -1] * 0.1 / np.linalg.norm(u[:3, -1])
    q_dot = dls_joint_velocity(q, radial, lower=None, upper=None)
    assert np.abs(q_dot).max() < 3.0
    assert np.linalg.norm(jac @ q_dot) < 0.2 * np.linalg.norm(radial)
    # A tangential request is the pan joint's own motion and is nearly fully met.
    tangential = jac[:, 0] * 0.3
    q_dot = dls_joint_velocity(q, tangential, lower=None, upper=None)
    achieved = jac @ q_dot
    assert achieved @ tangential / (tangential @ tangential) > 0.95
    assert np.linalg.norm(achieved - tangential) < 0.1 * np.linalg.norm(tangential)


def test_straight_elbow_with_pure_radial_twist():
    q = singular_q("elbow")
    shoulder = np.array([0.0, 0.0, 0.15185])
    pos, _ = tool_pose(q)
    # Outward along the straightened arm (the wrist_1 / lift axes are horizontal).
    jac = jacobian(q)
    outward = (pos - jac[:3, 1] * 0.0) - shoulder
    outward /= np.linalg.norm(outward)
    twist = np.concatenate([0.1 * outward, np.zeros(3)])
    q_dot = dls_joint_velocity(q, twist, lower=None, upper=None)
    assert np.isfinite(q_dot).all()
    assert np.abs(q_dot).max() < 3.0


def test_wrist_2_zero_feasible_motion_still_moves():
    q = singular_q("wrist_2")
    jac = jacobian(q)
    twist = jac[:, 1] * 0.3
    q_dot = dls_joint_velocity(q, twist, lower=None, upper=None)
    assert (jac @ q_dot) @ twist / (twist @ twist) > 0.95


def test_damping_is_continuous_and_zero_above_w0():
    assert damping_squared(1.0) == 0.0
    assert damping_squared(0.0) == pytest.approx(0.1**2)
    assert damping_squared(0.005 - 1e-12) == pytest.approx(0.0, abs=1e-9)


def test_velocity_limit_scales_the_whole_vector():
    q = BENT
    twist = np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    free = dls_joint_velocity(q, twist, lower=None, upper=None)
    limit = 0.5 * np.abs(free).max()
    capped = dls_joint_velocity(
        q, twist, max_joint_velocity=limit, lower=None, upper=None
    )
    assert np.abs(capped).max() == pytest.approx(limit)
    assert capped == pytest.approx(free * 0.5)


def test_joint_limits_only_stop_motion_into_the_limit():
    lower, upper = DEFAULT_LOWER, DEFAULT_UPPER
    q = np.array([0.0, -0.05, 0.0, 0.0, 2.6, -1.0])
    q_dot = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    out = limit_joint_velocity(q_dot, q, lower, upper, margin=0.15)
    assert out[0] == 1.0 and out[2] == 1.0 and out[3] == 1.0 and out[5] == 1.0
    assert 0.0 < out[1] < 0.5  # inside the margin of the upper limit
    assert 0.0 <= out[4] < 0.2
    # Moving away from the same limits is free.
    away = limit_joint_velocity(-q_dot, q, lower, upper, margin=0.15)
    assert away == pytest.approx(-q_dot)
    # At a limit nothing pushes further.
    at = limit_joint_velocity(q_dot, upper.copy(), lower, upper)
    assert (at == 0.0).all()


@pytest.mark.parametrize(
    "bad",
    [
        [np.nan] * 6,
        [np.inf, 0, 0, 0, 0, 0],
        [0.1, 0.1],
        "nonsense",
    ],
)
def test_bad_input_returns_zeros(bad):
    assert (dls_joint_velocity(BENT, bad) == 0.0).all()
    assert (dls_joint_velocity(bad, np.ones(6)) == 0.0).all()
    assert (
        dls_joint_velocity(BENT, np.ones(6), point_in_tool0=[np.nan] * 3) == 0
    ).all()


def test_sweep_is_finite_bounded_and_respects_joint_limits():
    rng = np.random.default_rng(7)
    dt = 0.01
    biggest_free = 0.0
    for _ in range(4000):
        q = random_pose(rng)
        twist = random_twist(rng)
        offset = rng.uniform(-0.1, 0.1, size=3)

        free = dls_joint_velocity(q, twist, offset, lower=None, upper=None)
        assert np.isfinite(free).all()
        biggest_free = max(biggest_free, np.abs(free).max())

        q_dot = dls_joint_velocity(q, twist, offset, max_joint_velocity=3.0)
        assert np.isfinite(q_dot).all()
        assert np.abs(q_dot).max() <= 3.0 + 1e-9
        nxt = q + q_dot * dt
        assert (nxt >= DEFAULT_LOWER - 1e-9).all()
        assert (nxt <= DEFAULT_UPPER + 1e-9).all()
    assert biggest_free < 20.0
    print(f"max |q_dot| unlimited over the sweep: {biggest_free:.2f} rad/s")


def test_sweep_near_limits_never_crosses():
    rng = np.random.default_rng(8)
    dt = 0.01
    for _ in range(2000):
        q = np.where(
            rng.random(6) < 0.5,
            DEFAULT_LOWER + rng.uniform(0.0, 0.1, 6),
            DEFAULT_UPPER - rng.uniform(0.0, 0.1, 6),
        )
        q = np.clip(q, DEFAULT_LOWER, DEFAULT_UPPER)
        q_dot = dls_joint_velocity(q, random_twist(rng), max_joint_velocity=3.0)
        nxt = q + q_dot * dt
        assert (nxt >= DEFAULT_LOWER - 1e-9).all()
        assert (nxt <= DEFAULT_UPPER + 1e-9).all()
