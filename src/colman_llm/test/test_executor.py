import logging

import pytest
from colman_llm.executor import FakeExecutor

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
