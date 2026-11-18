"""Break the motion server's dependencies and check it copes.

For every fault the server must stay up, end the goal in flight with a terminal result
(succeeded, aborted or canceled) in bounded time, and take the next goal once the fault
is gone.

The motion server plans in its own process (MoveItPy), so there is no separate
move_group to kill. The dependencies that can be broken from outside are the arm's
trajectory controller (the mock UR driver, killed or hung), the joint states, the
vacuum IO service and the tag TF.
"""

import pytest
from harness import wait_until

pytestmark = pytest.mark.fault_injection

# A fault must turn into a terminal result within this long. Trajectories on the mock
# arm take seconds; the execution monitor allows a multiple of that.
TERMINAL_S = 60


def moving_pick(client, frame="tag_2"):
    """Start a pick and return it once the arm is in motion towards the cube."""
    run = client.pick(frame)
    assert run.wait_accepted(), run
    assert run.wait_stage("approaching", 60), run
    client.wait_arm_moving()
    return run


# A trajectory that cannot be stopped wedges MoveIt's execution manager for good.
WEDGE_BUG = pytest.mark.xfail(
    strict=False,
    reason=(
        "BUG wedged-execution-manager: with the trajectory controller dead or hung "
        "mid-trajectory the execution manager's cancel never returns, the goal stays "
        "EXECUTING forever and the server never takes a goal again. Fixed by the "
        "'Bound the trajectory wait' commit on motion-fixes."
    ),
)


def assert_clean_failure(run):
    assert run.outcome in ("aborted", "canceled"), run
    assert run.message, f"no message explaining the failure: {run}"


def assert_recovers(rig):
    """After a fault is gone the server does a whole pick and place again."""
    rig.recover()
    client = rig.client
    run = client.pick("tag_2").wait_done()
    assert run.outcome == "succeeded", run
    run = client.place("tag_1").wait_done()
    assert run.outcome == "succeeded", run


@WEDGE_BUG
def test_driver_killed_mid_trajectory(healthy):
    client = healthy.client
    run = moving_pick(client)
    healthy.stack.kill("driver")
    run.wait_done(TERMINAL_S)
    assert_clean_failure(run)
    # The server may have to restart to get its execution manager back; it comes up
    # again once the arm does.
    healthy.restart_driver()
    healthy.wait_for_server()
    assert_recovers(healthy)


@WEDGE_BUG
def test_driver_hung_mid_trajectory(healthy):
    client = healthy.client
    run = moving_pick(client)
    healthy.stack.pause("driver")
    try:
        run.wait_done(TERMINAL_S)
        assert_clean_failure(run)
    finally:
        healthy.stack.resume("driver")
    assert_recovers(healthy)


@WEDGE_BUG
def test_stop_unsticks_a_hung_controller(healthy):
    client = healthy.client
    run = moving_pick(client)
    healthy.stack.pause("driver")
    try:
        # /stop must answer even though the controller cannot.
        assert client.trigger("/stop", timeout_s=15).success
        run.wait_done(TERMINAL_S)
        assert run.outcome in ("aborted", "canceled"), run
    finally:
        healthy.stack.resume("driver")
    assert_recovers(healthy)


def test_goal_while_the_driver_is_down(healthy):
    client = healthy.client
    healthy.stack.kill("driver")
    run = client.move_to("Compact").wait_done(TERMINAL_S)
    assert run.outcome in ("aborted", "canceled"), run
    run = client.pick("tag_2").wait_done(TERMINAL_S)
    assert_clean_failure(run)
    healthy.restart_driver()
    assert_recovers(healthy)


def test_sigterm_mid_goal_aborts_the_goal_and_exits(healthy):
    """Shutting the server down mid-pick must end the goal, halt the arm and exit."""
    import signal

    client = healthy.client
    run = moving_pick(client)
    healthy.stack.signal("motion_server", signal.SIGTERM)
    run.wait_done(TERMINAL_S)
    assert run.outcome in ("aborted", "canceled"), run
    process = healthy.stack.procs["motion_server"]
    wait_until(lambda: process.poll() is not None, 30, "the server to exit")
    client.wait_arm_still()
    # A new server comes up and works.
    healthy.restart_server()
    assert_recovers(healthy)


def test_joint_states_dropped_mid_trajectory(healthy):
    client = healthy.client
    run = moving_pick(client)
    client.pause_joint_states()
    try:
        run.wait_done(TERMINAL_S)
        # Without joint states the server cannot know where the arm is: it may well
        # finish the motion already in flight, but it must not hang.
        assert run.outcome in ("succeeded", "aborted", "canceled"), run
        if run.outcome != "succeeded":
            assert run.message, run
    finally:
        client.pause_joint_states(False)
    healthy.settle()
    assert_recovers(healthy)


def test_goal_sent_while_joint_states_are_down(healthy):
    client = healthy.client
    client.pause_joint_states()
    try:
        run = client.move_to("Compact").wait_done(TERMINAL_S)
        assert run.outcome in ("aborted", "canceled", "succeeded"), run
        run = client.pick("tag_2").wait_done(TERMINAL_S)
        assert run.outcome in ("aborted", "canceled", "succeeded"), run
        if run.outcome == "aborted":
            assert run.message, run
    finally:
        client.pause_joint_states(False)
    healthy.settle()
    assert_recovers(healthy)


def test_vacuum_grasp_fails(healthy):
    client = healthy.client
    client.io_mode("fail")
    run = client.pick("tag_2").wait_done()
    assert run.outcome == "aborted", run
    assert "grasp" in run.message.lower(), run
    client.io_mode("ok")
    assert_recovers(healthy)


def test_vacuum_grasp_times_out(healthy):
    client = healthy.client
    client.io_mode("hang")
    run = client.pick("tag_2").wait_done(TERMINAL_S)
    assert run.outcome == "aborted", run
    assert "grasp" in run.message.lower(), run
    client.io_mode("ok")
    assert_recovers(healthy)


def test_vacuum_release_fails_during_place(healthy):
    client = healthy.client
    run = client.pick("tag_2").wait_done()
    assert run.outcome == "succeeded", run
    client.io_mode("fail")
    run = client.place("tag_1").wait_done()
    assert run.outcome == "aborted", run
    assert "release" in run.message.lower(), run
    # The cube is still on the tip, so the server must still say it holds one.
    state = client.server_state()
    assert state["holding"], state
    client.io_mode("ok")
    assert_recovers(healthy)


def test_vacuum_release_times_out_during_place(healthy):
    client = healthy.client
    assert client.pick("tag_2").wait_done().outcome == "succeeded"
    client.io_mode("hang")
    run = client.place("tag_1").wait_done(TERMINAL_S)
    assert run.outcome == "aborted", run
    assert "release" in run.message.lower(), run
    client.io_mode("ok")
    assert_recovers(healthy)


def test_tag_never_visible(healthy):
    client = healthy.client
    client.hide_tags(2)
    run = client.pick("tag_2").wait_done(TERMINAL_S)
    assert run.outcome == "aborted", run
    assert "not visible" in run.message, run
    assert_recovers(healthy)


def test_tag_vanishes_while_the_goal_is_running(healthy):
    client = healthy.client
    run = client.pick("tag_2")
    assert run.wait_accepted()
    assert run.wait_stage("approaching", 60), run
    client.hide_tags(2)  # the pose was read already, so the pick can still finish
    run.wait_done(TERMINAL_S)
    assert run.outcome in ("succeeded", "aborted", "canceled"), run
    if run.outcome != "succeeded":
        assert run.message, run
    assert_recovers(healthy)


def test_place_target_tag_missing(healthy):
    client = healthy.client
    assert client.pick("tag_2").wait_done().outcome == "succeeded"
    client.hide_tags(1)
    run = client.place("tag_1").wait_done(TERMINAL_S)
    assert run.outcome == "aborted", run
    assert "not visible" in run.message, run
    # Still holding the cube; placing it by position works.
    assert client.server_state()["holding"]
    client.reset_tags()
    run = client.place("tag_1").wait_done(TERMINAL_S)
    assert run.outcome == "succeeded", run


@WEDGE_BUG
def test_server_survives_the_driver_coming_back_repeatedly(healthy):
    client = healthy.client
    for _ in range(2):
        run = moving_pick(client)
        healthy.stack.kill("driver")
        run.wait_done(TERMINAL_S)
        assert_clean_failure(run)
        healthy.restart_driver()
        healthy.recover()
        wait_until(lambda: client.server_state(), 10, "the server")
    assert_recovers(healthy)
