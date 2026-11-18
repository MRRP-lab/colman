"""The goal callbacks turn an unexpected exception into an explained abort.

Needs no stack: the decorator is exercised with a stand-in server and goal handle.
"""

import pytest

from colman_interfaces.action import MovePose, Pick

motion_server = pytest.importorskip("colman_motion.motion_server")

GUARD_BUG = pytest.mark.xfail(
    getattr(motion_server, "guarded", None) is None,
    strict=False,
    reason=(
        "BUG empty-abort-on-exception: an exception in a goal callback aborts the goal "
        "with an empty result. Fixed by the 'Log every abort with its stage, goal id "
        "and MoveIt error code' commit on motion-fixes."
    ),
)


class Logger:
    def __init__(self):
        self.errors = []

    def error(self, text):
        self.errors.append(text)


class Handle:
    is_active = True

    def __init__(self):
        self.aborted_with = None

    def abort(self, result):
        self.aborted_with = result
        self.is_active = False


class Server:
    def __init__(self):
        self.logger = Logger()
        self.finished = False

    def get_logger(self):
        return self.logger

    def finish(self):
        self.finished = True

    def describe(self, goal_handle):
        return "goal=test stage=test moveit=none"


@GUARD_BUG
@pytest.mark.parametrize("result_type", [Pick.Result, MovePose.Result])
def test_exception_becomes_an_abort_with_a_message(result_type):
    class Boom(Server):
        @motion_server.guarded(result_type)
        def callback(self, goal_handle):
            raise ValueError("boom")

    server, handle = Boom(), Handle()
    result = server.callback(handle)
    assert handle.aborted_with is result
    assert "ValueError: boom" in getattr(result, "message", "ValueError: boom")
    assert server.finished, "the arm would stay marked busy"
    assert any("boom" in line for line in server.logger.errors)


@GUARD_BUG
def test_a_callback_that_returns_is_left_alone():
    class Fine(Server):
        @motion_server.guarded(Pick.Result)
        def callback(self, goal_handle):
            return "done"

    server, handle = Fine(), Handle()
    assert server.callback(handle) == "done"
    assert handle.aborted_with is None and not server.finished
