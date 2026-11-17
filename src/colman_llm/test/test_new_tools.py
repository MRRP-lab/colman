"""pick_and_place with the real cubes, release_cube, go_to_pose and report_state."""

import logging

import pytest
from colman_llm.executor import FakeExecutor, PickAndPlaceExecutor

LOG = logging.getLogger("test")


def make(pick=None, place=None, release=(True, "off"), move=(True, "ok"), stop=True):
    """Executor with scripted arm results. Returns (executor, calls)."""
    calls = []
    picks, places = list(pick or []), list(place or [])

    def do_pick(frame):
        calls.append(("pick", frame))
        return picks.pop(0) if picks else (True, "ok")

    def do_place(frame):
        calls.append(("place", frame))
        return places.pop(0) if places else (True, "ok")

    def do_release():
        calls.append(("release",))
        return release

    def do_move(pose):
        calls.append(("move_to", pose))
        return move

    executor = PickAndPlaceExecutor(
        do_pick,
        do_place,
        LOG,
        stop=(lambda: (True, "stopped")) if stop else None,
        resume=(lambda: (True, "resumed")) if stop else None,
        release=do_release,
        move_to=do_move,
    )
    return executor, calls


def pap(executor, obj, dest):
    return executor.run_tool("pick_and_place", {"object": obj, "destination": dest})


# ------------------------------------------------------------------ pick_and_place
def test_any_loose_cube_goes_on_any_cube():
    executor, calls = make()
    ok, message = pap(executor, "blue cube 3", "white cube 1")
    assert ok and "blue cube 3" in message and "white cube 1" in message
    assert calls == [("pick", "tag_7"), ("place", "tag_1")]


def test_a_manual_move_is_remembered_by_stack_cube():
    executor, calls = make()
    pap(executor, "white cube 2", "white cube 1")
    executor.run_tool("stack_cube", {"color": "white", "count": 1})
    # white cube 3 goes on the cube that is now on top, not on the base again.
    assert calls[-2:] == [("pick", "tag_3"), ("place", "tag_2")]


def test_blocked_moves_never_reach_the_arm():
    executor, calls = make()
    pap(executor, "white cube 2", "white cube 1")
    for obj, dest, why in [
        ("white cube 2", "white cube 1", "already on"),
        ("white cube 3", "white cube 1", "already on top"),
        ("white cube 3", "white cube 3", "itself"),
    ]:
        ok, message = pap(executor, obj, dest)
        assert not ok and why in message
    del calls[:2]
    assert calls == []


def test_a_covered_cube_cannot_be_picked_up():
    executor, calls = make()
    pap(executor, "white cube 3", "white cube 2")
    del calls[:]
    ok, message = pap(executor, "white cube 2", "blue cube 1")
    assert not ok and "white cube 3 is on top of the white cube 2" in message
    assert calls == []


def test_stacks_stop_at_the_height_limit():
    executor, calls = make()
    for obj, dest in [
        ("white cube 2", "white cube 1"),
        ("white cube 3", "white cube 2"),
        ("white cube 4", "white cube 3"),
    ]:
        assert pap(executor, obj, dest)[0]
    del calls[:]
    ok, message = pap(executor, "blue cube 2", "white cube 4")
    assert not ok and "3 cubes high" in message and calls == []


def test_failed_pick_changes_nothing():
    executor, calls = make(pick=[(False, "not visible")])
    ok, message = pap(executor, "white cube 2", "white cube 1")
    assert not ok and "not visible" in message
    assert calls == [("pick", "tag_2")]
    assert pap(executor, "white cube 2", "white cube 1")[0]  # the retry is allowed


def test_failed_place_leaves_the_cube_held_until_released():
    executor, calls = make(place=[(False, "not visible")])
    ok, message = pap(executor, "white cube 2", "white cube 1")
    assert not ok and "release" in message.lower()
    # Nothing else may move while the cube is still in the gripper.
    ok, message = pap(executor, "white cube 3", "white cube 1")
    assert not ok and "still holding the white cube 2" in message
    ok, message = executor.run_tool("go_to_pose", {"pose": "home"})
    assert not ok and "holding" in message
    assert calls == [("pick", "tag_2"), ("place", "tag_1")]
    ok, message = executor.run_tool("release_cube", {})
    assert ok and "white cube 2" in message
    # It is loose on the table again, so it can be picked normally.
    assert pap(executor, "white cube 2", "white cube 1")[0]


def test_stack_cube_failed_place_also_holds_the_cube():
    executor, _ = make(place=[(False, "no")])
    ok, _ = executor.run_tool("stack_cube", {"color": "white", "count": 1})
    assert not ok
    ok, message = pap(executor, "blue cube 2", "blue cube 1")
    assert not ok and "still holding the white cube 2" in message


def test_pick_and_place_is_rejected_while_busy():
    holder = {}

    def pick(frame):
        holder["second"] = pap(holder["executor"], "white cube 3", "white cube 1")
        return True, "ok"

    holder["executor"] = PickAndPlaceExecutor(pick, lambda f: (True, "ok"), LOG)
    assert pap(holder["executor"], "white cube 2", "white cube 1")[0]
    ok, message = holder["second"]
    assert not ok and "busy" in message


# ------------------------------------------------------------------- release_cube
def test_release_with_nothing_held_still_turns_the_vacuum_off():
    executor, calls = make()
    ok, message = executor.run_tool("release_cube", {})
    assert ok and "wasn't holding" in message and calls == [("release",)]


def test_release_failure_is_reported():
    executor, _ = make(release=(False, "no service"))
    ok, message = executor.run_tool("release_cube", {})
    assert not ok and message == "no service"


def test_release_takes_no_arguments():
    executor, calls = make()
    ok, _ = executor.run_tool("release_cube", {"force": True})
    assert not ok and calls == []


def test_release_is_refused_while_the_arm_is_moving():
    holder = {}

    def place(frame):
        holder["release"] = holder["executor"].run_tool("release_cube", {})
        return True, "ok"

    calls = []
    holder["executor"] = PickAndPlaceExecutor(
        lambda f: (True, "ok"),
        place,
        LOG,
        release=lambda: calls.append("release") or (True, "off"),
    )
    pap(holder["executor"], "white cube 2", "white cube 1")
    ok, message = holder["release"]
    assert not ok and "busy" in message and calls == []


# -------------------------------------------------------------------- go_to_pose
@pytest.mark.parametrize(
    "pose, srdf",
    [
        ("home", "home"),
        ("compact", "Compact"),
    ],
)
def test_named_poses_map_to_the_srdf_names(pose, srdf):
    executor, calls = make()
    ok, message = executor.run_tool("go_to_pose", {"pose": pose})
    assert ok and pose in message and calls == [("move_to", srdf)]


@pytest.mark.parametrize(
    "args",
    [
        {"pose": "Compact"},  # the raw name is not a spoken pose
        {"pose": "zero"},
        {"pose": "Up"},
        {"pose": 3},
        {},
        {"pose": "home", "speed": 1},
        "home",
    ],
)
def test_unknown_poses_never_reach_the_arm(args):
    executor, calls = make()
    ok, _ = executor.run_tool("go_to_pose", args)
    assert not ok and calls == []


def test_move_failure_is_reported():
    executor, _ = make(move=(False, "The arm rejected the move."))
    ok, message = executor.run_tool("go_to_pose", {"pose": "compact"})
    assert not ok and "compact" in message and "rejected" in message


def test_go_to_pose_is_refused_while_busy():
    holder = {}

    def pick(frame):
        holder["pose"] = holder["executor"].run_tool("go_to_pose", {"pose": "home"})
        return True, "ok"

    holder["executor"] = PickAndPlaceExecutor(
        pick, lambda f: (True, "ok"), LOG, move_to=lambda p: (True, "ok")
    )
    pap(holder["executor"], "white cube 2", "white cube 1")
    ok, message = holder["pose"]
    assert not ok and "busy" in message


# ------------------------------------------------------------------ report_state
def test_report_state_on_a_fresh_table():
    executor, calls = make()
    ok, message = executor.run_tool("report_state", {})
    assert ok and calls == []
    assert "No cubes are stacked" in message
    assert "white cube 2" in message and "blue cube 4" in message
    assert "not holding anything" in message and "ready" in message


def test_report_state_follows_the_stacks_and_the_gripper():
    executor, _ = make(place=[(True, "ok"), (False, "no")])
    pap(executor, "white cube 2", "white cube 1")
    pap(executor, "blue cube 2", "white cube 2")  # place fails, still holding
    _, message = executor.run_tool("report_state", {})
    assert "the white cube 1 has white cube 2 on top" in message
    assert "holding the blue cube 2" in message


def test_report_state_knows_about_stop_and_resume():
    executor, _ = make()
    executor.run_tool("stop_robot", {})
    assert "The arm is stopped" in executor.run_tool("report_state", {})[1]
    executor.run_tool("resume_robot", {})
    assert "ready" in executor.run_tool("report_state", {})[1]
    executor.set_stopped(True)  # a spoken stop heard by the node
    assert "The arm is stopped" in executor.run_tool("report_state", {})[1]


def test_report_state_takes_no_arguments():
    executor, _ = make()
    assert not executor.run_tool("report_state", {"verbose": True})[0]


# --------------------------------------------------------------------- dry run
def test_fake_executor_runs_every_tool():
    fake = FakeExecutor(LOG)
    assert fake.run_tool(
        "pick_and_place", {"object": "blue cube 2", "destination": "blue cube 1"}
    )[0]
    assert fake.run_tool("go_to_pose", {"pose": "home"})[0]
    assert fake.run_tool("release_cube", {})[0]
    assert fake.run_tool("report_state", {})[0]
