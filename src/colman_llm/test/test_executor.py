import logging

import pytest
from colman_llm.executor import FakeExecutor, PickAndPlaceExecutor
from colman_llm.fake_arm import FakeArm

LOG = logging.getLogger("test")
GOOD = {"object": "white cube 3", "destination": "white cube 2"}


def test_valid_call_gets_canned_success():
    ok, message = FakeExecutor(LOG).run_tool("pick_and_place", GOOD)
    assert ok
    assert "white cube 3" in message and "white cube 2" in message


@pytest.mark.parametrize(
    "args",
    [
        {"object": "green cube 2", "destination": "white cube 2"},
        {"object": "white cube 3", "destination": "kitchen"},
        {"object": "white cube 3"},
        {**GOOD, "extra": "x"},
        {"object": "tag_3", "destination": "tag_1"},
        {"object": 3, "destination": ["white cube 2"]},
        {"object": "white cube 2", "destination": "white cube 2"},  # onto itself
        "white cube 3",
    ],
)
def test_unknown_or_malformed_arguments_are_rejected(args):
    ok, message = FakeExecutor(LOG).run_tool("pick_and_place", args)
    assert not ok and message


def test_unknown_tool_is_rejected():
    ok, _ = FakeExecutor(LOG).run_tool("move_to_xyz", GOOD)
    assert not ok


def make_executor(pick=(True, "picked"), place=(True, "placed")):
    arm = FakeArm(LOG, [pick], [place])
    return arm.executor(), arm.calls


def test_pick_then_place_with_mapped_frames():
    executor, calls = make_executor()
    ok, message = executor.run_tool("pick_and_place", GOOD)
    assert ok and "white cube 3" in message
    assert calls == [("pick", "tag_3"), ("place", "tag_2")]


def test_pick_failure_aborts_before_place():
    executor, calls = make_executor(pick=(False, "tag_3 not visible"))
    ok, message = executor.run_tool("pick_and_place", GOOD)
    assert not ok and "tag_3 not visible" in message
    assert calls == [("pick", "tag_3")]


def test_place_failure_is_reported_after_a_successful_pick():
    executor, calls = make_executor(place=(False, "tag_2 not visible"))
    ok, message = executor.run_tool("pick_and_place", GOOD)
    assert not ok
    assert "Picked up the white cube 3" in message and "tag_2 not visible" in message
    assert [c[0] for c in calls] == ["pick", "place"]


def test_invalid_arguments_never_reach_the_arm():
    executor, calls = make_executor()
    ok, _ = executor.run_tool(
        "pick_and_place", {"object": "tag_3", "destination": "tag_1"}
    )
    assert not ok and calls == []


def test_second_pick_and_place_while_busy_is_rejected():
    holder = {}

    def pick(frame):
        # A second call arrives while the first is still picking.
        holder["second"] = holder["executor"].run_tool("pick_and_place", GOOD)
        return arm.do_pick(frame)

    arm = FakeArm(LOG)
    holder["executor"] = PickAndPlaceExecutor(
        pick, arm.do_place, LOG, arm.world.view, sleep=lambda s: None
    )
    ok, _ = holder["executor"].run_tool("pick_and_place", GOOD)
    assert ok
    second_ok, second_message = holder["second"]
    assert not second_ok and "busy" in second_message


def make_with_stop(stop=(True, "Stopped"), resume=(True, "Resumed")):
    calls = []

    def do_stop():
        calls.append("stop")
        return stop

    def do_resume():
        calls.append("resume")
        return resume

    arm = FakeArm(LOG)
    executor = PickAndPlaceExecutor(
        arm.do_pick,
        arm.do_place,
        LOG,
        arm.world.view,
        stop=do_stop,
        resume=do_resume,
    )
    return executor, calls


def test_stop_and_resume_tools_call_the_services():
    executor, calls = make_with_stop()
    assert executor.run_tool("stop_robot", {})[0]
    assert executor.run_tool("resume_robot", {})[0]
    assert calls == ["stop", "resume"]


def test_service_failure_is_an_error():
    executor, _ = make_with_stop(stop=(False, "no service"))
    ok, message = executor.run_tool("stop_robot", {})
    assert not ok and message == "no service"


def test_stop_takes_no_arguments():
    executor, calls = make_with_stop()
    ok, _ = executor.run_tool("stop_robot", {"force": True})
    assert not ok and calls == []


def test_stop_works_while_a_pick_and_place_holds_the_lock():
    holder = {}

    def pick(frame):
        holder["stop"] = holder["executor"].run_tool("stop_robot", {})
        return True, "picked"

    arm = FakeArm(LOG)

    def pick_then_stop(frame):
        pick(frame)
        return arm.do_pick(frame)

    executor = PickAndPlaceExecutor(
        pick_then_stop,
        arm.do_place,
        LOG,
        arm.world.view,
        stop=lambda: (True, "Stopped"),
        sleep=lambda s: None,
    )
    holder["executor"] = executor
    executor.run_tool("pick_and_place", GOOD)
    assert holder["stop"][0]


def test_fake_executor_handles_stop_and_resume():
    fake = FakeExecutor(LOG)
    assert fake.run_tool("stop_robot", {})[0]
    assert fake.run_tool("resume_robot", {})[0]


def make_stacker(pick_results=None, place_results=None):
    """Executor whose pick and place results come from lists (default: success)."""
    arm = FakeArm(LOG, pick_results, place_results)
    return arm.executor(), arm.calls


def test_stack_four_white_cubes_builds_one_tower_in_any_order():
    executor, calls = make_stacker()
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 4})
    assert ok and "The white cubes are stacked, 4 high" in message
    picks = [f for kind, f in calls if kind == "pick"]
    places = [f for kind, f in calls if kind == "place"]
    assert len(picks) == 3 and len(set(picks)) == 3
    # Nothing is a base: the first cube stays where it lies, whichever it is, and each
    # cube goes on the one placed before it.
    anchor = ({"tag_1", "tag_2", "tag_3", "tag_4"} - set(picks)).pop()
    assert places == [anchor, picks[0], picks[1]]


def test_stacking_continues_from_the_tallest_stack_across_calls():
    executor, calls = make_stacker()
    executor.run_tool("stack_cube", {"color": "blue", "count": 2})
    executor.run_tool("stack_cube", {"color": "blue", "count": 3})
    picks = [c for c in calls if c[0] == "pick"]
    places = [c for c in calls if c[0] == "place"]
    assert len(picks) == 2 and len(places) == 2
    assert places[1][1] == picks[0][1]  # the second cube went on the first one placed


def test_asking_for_what_is_already_stacked_does_nothing():
    executor, calls = make_stacker()
    executor.run_tool("stack_cube", {"color": "white", "count": 3})
    n = len(calls)
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 2})
    assert ok and "already stacked" in message and len(calls) == n


def test_failed_pick_stops_the_run_and_reports_progress():
    executor, calls = make_stacker(pick_results=[(True, "ok"), (False, "not visible")])
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 4})
    assert not ok
    assert "Placed 1 white cube." in message and "Could not pick up" in message
    assert "not visible" in message and "2 high" in message


def test_failed_place_does_not_count_the_cube_as_stacked():
    executor, calls = make_stacker(place_results=[(False, "tag_1 not visible")])
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 2})
    assert not ok and "Placed 0 white cubes" in message and "1 high" in message
    # The arm still holds the cube, so nothing else is picked until it is let go.
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 2})
    assert not ok and "still holding" in message
    executor.run_tool("release_cube", {})
    executor.run_tool("stack_cube", {"color": "white", "count": 2})
    assert calls[-2][0] == "pick" and calls[-1][0] == "place"


@pytest.mark.parametrize(
    "args",
    [
        {"color": "red", "count": 1},
        {"color": "white", "count": 0},
        {"color": "white", "count": 1},
        {"color": "white", "count": 5},
        {"color": "white", "count": 1.5},
        {"color": "white", "count": True},
        {"color": "white"},
        {"color": "white", "count": 1, "extra": 1},
        "white",
    ],
)
def test_bad_stack_arguments_never_reach_the_arm(args):
    executor, calls = make_stacker()
    ok, _ = executor.run_tool("stack_cube", args)
    assert not ok and calls == []


def test_fake_executor_stacks_without_an_arm():
    fake = FakeExecutor(LOG)
    ok, message = fake.run_tool("stack_cube", {"color": "blue", "count": 3})
    assert ok and "The blue cubes are stacked, 3 high" in message


def test_each_placement_is_announced_as_it_happens():
    said = []
    arm = FakeArm(LOG)
    executor = arm.executor(announce=said.append)
    ok, message = executor.run_tool("stack_cube", {"color": "blue", "count": 4})
    assert ok
    assert said == [
        "I've placed blue cube 2 on blue cube 1.",
        "I've placed blue cube 3 on blue cube 2.",
        "I've placed blue cube 4 on blue cube 3.",
    ]
    assert "already announced aloud" in message


def test_a_failed_placement_is_not_announced():
    said = []
    arm = FakeArm(LOG, place=[(True, "ok"), (False, "tag_2 not visible")])
    executor = arm.executor(announce=said.append)
    ok, _ = executor.run_tool("stack_cube", {"color": "white", "count": 3})
    assert not ok
    assert len(said) == 2 and said[0].startswith("I've placed white cube ")
    assert said[1].endswith("but couldn't place it.")


def test_a_broken_announcer_does_not_stop_the_stacking():
    def broken(text):
        raise RuntimeError("no speaker")

    arm = FakeArm(LOG)
    ok, message = arm.executor(announce=broken).run_tool(
        "stack_cube", {"color": "white", "count": 3}
    )
    assert ok and "The white cubes are stacked, 3 high" in message


def test_without_an_announcer_the_message_does_not_mention_one():
    executor, _ = make_stacker()
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 2})
    assert ok and "announced" not in message


def test_a_failure_is_announced_the_moment_it_happens():
    said = []
    arm = FakeArm(LOG, pick=[(False, "not visible")])
    ok, message = arm.executor(announce=said.append).run_tool(
        "pick_and_place", {"object": "white cube 3", "destination": "blue cube 1"}
    )
    assert not ok and said == ["I couldn't pick up the white cube 3."]
