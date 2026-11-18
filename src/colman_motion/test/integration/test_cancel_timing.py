"""Cancel or /stop at each stage of a pick and a place.

Checks the goal state and the arm agree: after a cancel or stop the goal ends
canceled (or aborted for /stop), the arm halts promptly and stays halted, the vacuum
and the server's idea of holding a cube agree, and the next goal works. Note the halt
itself runs in a daemon thread, so it can lag the goal's result; that is what the
"stays halted" check is for.
"""

import time

import pytest
from action_msgs.srv import CancelGoal

slow = pytest.mark.slow

# How long after the request the arm may take to come to rest.
HALT_S = 4.0
# How long after the arm first looks still it must remain so (a late halt thread
# would move it again, or a trajectory that was not stopped would resume).
STAY_STILL_S = 2.0
POSITION_EPS = 5e-3

# (action, stage to cancel at, wait for the arm to be moving first)
PICK_STAGES = [
    ("pick", "detecting", False),
    ("pick", "approaching", False),  # straight after the stage: still planning
    ("pick", "approaching", True),  # the approach is under way
    ("pick", "lowering", True),
    ("pick", "grasping", False),
    ("pick", "retreating", True),
]
PLACE_STAGES = [
    ("place", "detecting", False),
    ("place", "approaching", False),
    ("place", "approaching", True),
    ("place", "lowering", True),
    ("place", "releasing", False),
    ("place", "retreating", True),
]
FAST = {
    ("pick", "detecting", False),
    ("pick", "approaching", True),
    ("pick", "grasping", False),
}


def case_id(case):
    action, stage, moving = case
    return f"{action}-{stage}{'-moving' if moving else ''}"


def params():
    return [
        pytest.param(case, id=case_id(case), marks=[] if case in FAST else [slow])
        for case in PICK_STAGES + PLACE_STAGES
    ]


def positions(client):
    return dict(client.joint_positions())


def drift(a, b):
    return max(abs(a[j] - b[j]) for j in a if j in b)


def interrupt(rig, run, how):
    """Cancel the goal or hit /stop. Returns the time of the request."""
    client = rig.client
    requested = time.monotonic()
    if how == "cancel":
        assert run.cancel() == CancelGoal.Response.ERROR_NONE
    else:
        assert client.trigger("/stop").success
    return requested


def start_case(rig, action, stage, moving):
    client = rig.client
    if action == "place":
        assert client.pick("tag_2").wait_done().outcome == "succeeded"
        run = client.place("tag_1")
    else:
        run = client.pick("tag_2")
    assert run.wait_accepted(), run
    assert run.wait_stage(stage, 60), f"never reached {stage}: {run}"
    if moving:
        client.wait_arm_moving(10)
    return run


def check_consistency(rig, run, how, action):
    client = rig.client
    # The goal ends, and the arm comes to rest and stays there.
    run.wait_done(30)
    if how == "cancel":
        assert run.outcome in ("canceled", "succeeded"), run
    else:
        assert run.outcome in ("aborted", "canceled"), run
    state = rig.settle()
    # settle() only returns when the server is idle; the arm must be too.
    client.wait_arm_still(HALT_S)
    rest = positions(client)
    deadline = time.monotonic() + STAY_STILL_S
    while time.monotonic() < deadline:
        assert drift(rest, positions(client)) < POSITION_EPS, (
            f"arm moved again after the goal ended: {run}"
        )
        time.sleep(0.1)
    # The vacuum output and the server agree on whether a cube is held, and a cube
    # is only modelled on the tip while it is held.
    assert state["holding"] == client.vacuum_on(), (
        f"vacuum and server disagree: {state}"
    )
    assert state["world"] == ["table"], state
    assert state["attached"] == [] or state["holding"], state
    return state


@pytest.mark.parametrize("how", ["cancel", "stop"])
@pytest.mark.parametrize("case", params())
def test_interrupt_at_stage(healthy, case, how):
    action, stage, moving = case
    client = healthy.client
    run = start_case(healthy, action, stage, moving)
    interrupt(healthy, run, how)
    still = wait_for_halt(client)
    assert still < HALT_S, f"arm still moving {still:.1f}s after the {how}"
    state = check_consistency(healthy, run, how, action)
    if how == "stop":
        refused = client.move_to("home")
        assert refused.wait_accepted() is False, "goal accepted while stopped"
        assert client.trigger("/resume").success
    # After either, the server takes new work: let go if needed and move.
    if state["holding"]:
        healthy.release()
    follow = client.move_to("home").wait_done()
    assert follow.outcome == "succeeded", follow
    final = healthy.settle()
    assert final["world"] == ["table"] and final["attached"] == [], final


def wait_for_halt(client):
    """Seconds until the arm is first seen at rest."""
    started = time.monotonic()
    client.wait_arm_still(HALT_S + 10)
    return time.monotonic() - started


def test_cancel_when_nothing_is_running(healthy):
    client = healthy.client
    run = client.move_to("home")
    run.wait_done()
    # Cancelling a finished goal is refused, and changes nothing.
    assert run.cancel() != CancelGoal.Response.ERROR_NONE
    assert client.move_to("Compact").wait_done().outcome == "succeeded"


def test_stop_when_nothing_is_running_then_resume(healthy):
    client = healthy.client
    assert client.trigger("/stop").success
    assert client.move_to("home").wait_accepted() is False
    assert client.trigger("/resume").success
    assert client.move_to("home").wait_done().outcome == "succeeded"


def test_cancel_repeated_does_not_wedge_the_server(healthy):
    client = healthy.client
    run = client.pick("tag_2")
    assert run.wait_stage("approaching", 60)
    for _ in range(3):
        run.cancel()
    run.wait_done(30)
    assert run.outcome == "canceled", run
    healthy.settle()
    assert client.move_to("home").wait_done().outcome == "succeeded"
