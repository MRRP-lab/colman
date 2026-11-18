"""Poses and tags the arm cannot or should not go to.

Each one must end in a clean abort with a message that says why (or, where the pose is
borderline, a success), never a crash or a hang, and leave the planning scene as it
found it. The next goal must then work.
"""

import re

import pytest

NAN = float("nan")
INF = float("inf")
DOWN = (1.0, 0.0, 0.0, 0.0)

# name -> (xyz, quaternion, must_abort). Borderline poses may also succeed.
MOVE_POSE_CORPUS = {
    "far_out_of_reach": ((2.0, 0.0, 0.3), DOWN, True),
    "behind_the_base_out_of_reach": ((-0.9, 0.0, 0.3), DOWN, True),
    "far_below_the_table": ((0.3, 0.0, -0.4), DOWN, True),
    "inside_the_table": ((0.3, 0.0, -0.02), DOWN, True),
    "inside_the_base": ((0.0, 0.0, 0.1), DOWN, True),
    "nan_position": ((NAN, 0.0, 0.3), DOWN, True),
    "nan_orientation": ((0.3, 0.0, 0.3), (NAN, 0.0, 0.0, 1.0), True),
    "infinite_position": ((INF, 0.0, 0.3), DOWN, True),
    "huge_position": ((1e9, 1e9, 1e9), DOWN, True),
    "zero_quaternion": ((0.3, 0.0, 0.3), (0.0, 0.0, 0.0, 0.0), True),
    "unnormalised_quaternion": ((0.3, 0.0, 0.3), (2.0, 0.0, 0.0, 0.0), False),
    "straight_up_near_singularity": ((0.0, 0.0, 0.7), DOWN, False),
    "at_the_edge_of_reach": ((0.45, 0.0, 0.1), DOWN, False),
    "wrist_near_its_limit": ((0.3, 0.0, 0.3), (0.0, 1.0, 0.0, 0.0), False),
}

ZERO_QUATERNION_BUG = pytest.mark.xfail(
    strict=False,
    reason=(
        "BUG zero-quaternion-moves: a (0, 0, 0, 0) orientation is normalised to "
        "something arbitrary and the arm moves there. Fixed by the 'Reject a "
        "move_pose with a zero-length orientation' commit on motion-fixes."
    ),
)
MARKS = {"zero_quaternion": ZERO_QUATERNION_BUG}

# name -> (tag pose [x y z qx qy qz qw], must_abort)
TAG_CORPUS = {
    "tag_out_of_reach": ([1.5, 0.0, 0.039, 0.0, 0.0, 0.0, 1.0], True),
    "tag_under_the_table": ([0.3, 0.0, -0.5, 0.0, 0.0, 0.0, 1.0], True),
    "tag_inside_the_base": ([0.0, 0.0, 0.039, 0.0, 0.0, 0.0, 1.0], True),
    "tag_with_nan_position": ([NAN, 0.0, 0.039, 0.0, 0.0, 0.0, 1.0], True),
    "tag_with_zero_quaternion": ([0.3, 0.0, 0.039, 0.0, 0.0, 0.0, 0.0], False),
    "tag_tipped_over": ([0.3, 0.1, 0.039, 0.7071, 0.0, 0.0, 0.7071], False),
}

PLACE_POSITIONS = {
    "out_of_reach": ((3.0, 0.0, 0.1), True),
    "below_the_table": ((0.3, 0.0, -0.5), True),
    "nan": ((NAN, NAN, NAN), True),
    "infinite": ((INF, 0.0, 0.1), True),
}


def assert_scene_is_clean(rig, allow_held=False):
    """No stale world objects; an attached cube only while one is held."""
    state = rig.client.server_state()
    assert state["world"] == ["table"], f"stale collision objects: {state}"
    if state["attached"]:
        assert allow_held and state["holding"], f"stale attached object: {state}"
    return state


def assert_aborted_with_reason(run):
    assert run.outcome == "aborted", run
    assert len(run.message) > 8, f"abort without a useful message: {run}"
    assert "traceback" not in run.message.lower(), run


def assert_next_goal_works(rig):
    run = rig.client.move_to("Compact").wait_done()
    assert run.outcome == "succeeded", f"server did not recover: {run}"


@pytest.mark.parametrize(
    "name", [pytest.param(n, marks=MARKS.get(n, ())) for n in MOVE_POSE_CORPUS]
)
def test_move_pose(healthy, name):
    xyz, quat, must_abort = MOVE_POSE_CORPUS[name]
    run = healthy.client.move_pose(xyz, quat).wait_done()
    if must_abort:
        assert_aborted_with_reason(run)
    else:
        assert run.outcome in ("succeeded", "aborted"), run
        if run.outcome == "aborted":
            assert_aborted_with_reason(run)
    assert_scene_is_clean(healthy)
    assert_next_goal_works(healthy)


@pytest.mark.parametrize("name", TAG_CORPUS)
def test_pick_at_awkward_tag(healthy, name):
    pose, must_abort = TAG_CORPUS[name]
    healthy.client.set_tag(2, pose)
    run = healthy.client.pick("tag_2").wait_done()
    if must_abort:
        assert_aborted_with_reason(run)
    else:
        assert run.outcome in ("succeeded", "aborted"), run
        if run.outcome == "aborted":
            assert_aborted_with_reason(run)
    state = assert_scene_is_clean(healthy)
    assert state["holding"] == (run.outcome == "succeeded"), state
    healthy.client.reset_tags()
    assert_next_goal_works(healthy)


def test_pick_of_a_tag_that_does_not_exist(healthy):
    run = healthy.client.pick("tag_99").wait_done()
    assert_aborted_with_reason(run)
    assert "tag_99" in run.message, run
    assert_scene_is_clean(healthy)


def test_pick_with_no_frame(healthy):
    run = healthy.client.pick("").wait_done()
    assert_aborted_with_reason(run)
    assert_scene_is_clean(healthy)


@pytest.mark.parametrize("name", PLACE_POSITIONS)
def test_place_at_awkward_position(healthy, name):
    position, must_abort = PLACE_POSITIONS[name]
    client = healthy.client
    assert client.pick("tag_2").wait_done().outcome == "succeeded"
    run = client.place(position=position).wait_done()
    if must_abort:
        assert_aborted_with_reason(run)
    # The cube is still held, so it may still be modelled on the tip, and only then.
    assert_scene_is_clean(healthy, allow_held=True)
    assert client.server_state()["holding"] == (run.outcome != "succeeded")


def test_place_with_nothing_to_place_at(healthy):
    client = healthy.client
    assert client.pick("tag_2").wait_done().outcome == "succeeded"
    run = client.place().wait_done()
    assert_aborted_with_reason(run)
    assert "location" in run.message.lower() or "position" in run.message.lower(), run


def test_place_while_not_holding(healthy):
    run = healthy.client.place("tag_1").wait_done()
    assert_aborted_with_reason(run)
    assert "holding" in run.message.lower(), run
    assert_scene_is_clean(healthy)


def test_pick_while_already_holding(healthy):
    client = healthy.client
    assert client.pick("tag_2").wait_done().outcome == "succeeded"
    run = client.pick("tag_3").wait_done()
    assert_aborted_with_reason(run)
    assert "holding" in run.message.lower(), run


LOG_BUG = pytest.mark.xfail(
    strict=False,
    reason=(
        "BUG unexplained-aborts: aborts are logged without the goal, stage or MoveIt "
        "error code. Fixed by the 'Log every abort with its stage, goal id and MoveIt "
        "error code' commit on motion-fixes."
    ),
)


@LOG_BUG
def test_every_abort_is_logged_with_goal_stage_and_moveit_error(healthy):
    log = healthy.stack.log_dir / "motion_server.log"
    start = log.stat().st_size
    run = healthy.client.pick("tag_99").wait_done()
    assert run.outcome == "aborted", run
    far = healthy.client.move_pose((2.0, 0.0, 0.3)).wait_done()
    assert far.outcome == "aborted", far
    assert "MoveIt error" in far.message, far
    with open(log, errors="replace") as handle:
        handle.seek(start)
        aborts = [line for line in handle if "Abort:" in line]
    assert len(aborts) >= 2, aborts
    for line in aborts:
        assert re.search(r"\[goal=[0-9a-f]{8} stage=\S+ moveit=.+\]", line), line
