import logging

import pytest
from colman_llm.executor import FakeExecutor, PickAndPlaceExecutor

LOG = logging.getLogger("test")
GOOD = {"object": "red block", "destination": "left tray"}


def test_valid_call_gets_canned_success():
    ok, message = FakeExecutor(LOG).run_tool("pick_and_place", GOOD)
    assert ok
    assert "red block" in message and "left tray" in message


@pytest.mark.parametrize(
    "args",
    [
        {"object": "green block", "destination": "left tray"},
        {"object": "red block", "destination": "kitchen"},
        {"object": "red block"},
        {**GOOD, "extra": "x"},
        {"object": "tag_3", "destination": "tag_1"},
        {"object": 3, "destination": ["left tray"]},
        "red block",
    ],
)
def test_unknown_or_malformed_arguments_are_rejected(args):
    ok, message = FakeExecutor(LOG).run_tool("pick_and_place", args)
    assert not ok and message


def test_unknown_tool_is_rejected():
    ok, _ = FakeExecutor(LOG).run_tool("move_to_xyz", GOOD)
    assert not ok


def make_executor(pick=(True, "picked"), place=(True, "placed")):
    calls = []

    def do_pick(frame):
        calls.append(("pick", frame))
        return pick

    def do_place(frame):
        calls.append(("place", frame))
        return place

    return PickAndPlaceExecutor(do_pick, do_place, LOG), calls


def test_pick_then_place_with_mapped_frames():
    executor, calls = make_executor()
    ok, message = executor.run_tool("pick_and_place", GOOD)
    assert ok and "red block" in message
    assert calls == [("pick", "tag_3"), ("place", "tag_1")]


def test_pick_failure_aborts_before_place():
    executor, calls = make_executor(pick=(False, "tag_3 not visible"))
    ok, message = executor.run_tool("pick_and_place", GOOD)
    assert not ok and "tag_3 not visible" in message
    assert calls == [("pick", "tag_3")]


def test_place_failure_is_reported_after_a_successful_pick():
    executor, calls = make_executor(place=(False, "tag_1 not visible"))
    ok, message = executor.run_tool("pick_and_place", GOOD)
    assert not ok
    assert "Picked up the red block" in message and "tag_1 not visible" in message
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
        return True, "picked"

    holder["executor"] = PickAndPlaceExecutor(pick, lambda f: (True, "ok"), LOG)
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

    executor = PickAndPlaceExecutor(
        lambda f: (True, "p"),
        lambda f: (True, "p"),
        LOG,
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

    executor = PickAndPlaceExecutor(
        pick, lambda f: (True, "ok"), LOG, stop=lambda: (True, "Stopped")
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
    calls = []
    picks = list(pick_results or [])
    places = list(place_results or [])

    def pick(frame):
        calls.append(("pick", frame))
        return picks.pop(0) if picks else (True, "ok")

    def place(frame):
        calls.append(("place", frame))
        return places.pop(0) if places else (True, "ok")

    return PickAndPlaceExecutor(pick, place, LOG), calls


def test_stack_three_white_cubes_builds_the_tower_in_order():
    executor, calls = make_stacker()
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 3})
    assert ok and "Stacked 3 white cubes" in message
    assert calls == [
        ("pick", "tag_2"),
        ("place", "tag_1"),
        ("pick", "tag_3"),
        ("place", "tag_2"),
        ("pick", "tag_4"),
        ("place", "tag_3"),
    ]


def test_stacking_continues_from_the_current_top_across_calls():
    executor, calls = make_stacker()
    executor.run_tool("stack_cube", {"color": "blue", "count": 1})
    executor.run_tool("stack_cube", {"color": "blue", "count": 1})
    assert calls[-2:] == [("pick", "tag_7"), ("place", "tag_6")]


def test_asking_for_more_than_fits_stops_when_full():
    executor, calls = make_stacker()
    executor.run_tool("stack_cube", {"color": "white", "count": 2})
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 2})
    assert ok and "Stacked 1 white cube." in message and "full" in message
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 1})
    assert not ok and "already full" in message


def test_failed_pick_stops_the_run_and_reports_progress():
    executor, calls = make_stacker(pick_results=[(True, "ok"), (False, "not visible")])
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 3})
    assert not ok
    assert "Stacked 1 of 3" in message and "white cube 3" in message
    assert "not visible" in message and "1 on top" in message
    assert calls[-1] == ("pick", "tag_3")


def test_failed_place_does_not_count_the_cube_as_stacked():
    executor, calls = make_stacker(place_results=[(False, "tag_1 not visible")])
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 1})
    assert not ok and "0 on top" in message
    # The retry targets the same cube and the same spot.
    executor.run_tool("stack_cube", {"color": "white", "count": 1})
    assert calls[-2:] == [("pick", "tag_2"), ("place", "tag_1")]


@pytest.mark.parametrize(
    "args",
    [
        {"color": "red", "count": 1},
        {"color": "white", "count": 0},
        {"color": "white", "count": 4},
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
    ok, message = fake.run_tool("stack_cube", {"color": "blue", "count": 2})
    assert ok and "Stacked 2 blue cubes" in message
