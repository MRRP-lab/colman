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
