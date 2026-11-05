import logging
from types import SimpleNamespace

from colman_llm.action_call import ActionCall

LOG = logging.getLogger("test")


class Future:
    def __init__(self, value):
        self._value = value

    def result(self):
        return self._value

    def add_done_callback(self, callback):
        callback(self)


class Handle:
    def __init__(self, accepted=True, success=True, message="done"):
        self.accepted = accepted
        self.cancelled = False
        self._result = SimpleNamespace(
            result=SimpleNamespace(success=success, message=message)
        )

    def get_result_async(self):
        return Future(self._result)

    def cancel_goal_async(self):
        self.cancelled = True


class NeverDone:
    """A goal that is accepted but never produces a result."""

    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        pass


class FakeClient:
    def __init__(self, handle=None, server=True, hang=False):
        self.handle = handle
        self.server = server
        self.hang = hang

    def wait_for_server(self, timeout_sec):
        return self.server

    def send_goal_async(self, goal, feedback_callback=None):
        return Future(self.handle)


class HangingHandle(Handle):
    def get_result_async(self):
        return NeverDone()


def make(client):
    return ActionCall(client, "pick", LOG)


def test_success_returns_server_message():
    ok, message = make(FakeClient(Handle(message="Picked tag_3"))).call("goal", 1)
    assert ok and message == "Picked tag_3"


def test_server_failure_message_is_passed_on():
    ok, message = make(FakeClient(Handle(success=False, message="not visible"))).call(
        "goal", 1
    )
    assert not ok and message == "not visible"


def test_missing_server():
    ok, message = make(FakeClient(server=False)).call("goal", 1)
    assert not ok and "not running" in message


def test_rejected_goal_mentions_busy_and_resume():
    ok, message = make(FakeClient(Handle(accepted=False))).call("goal", 1)
    assert not ok and "busy" in message and "resumed" in message


def test_timeout_cancels_the_goal():
    handle = HangingHandle()
    ok, message = make(FakeClient(handle)).call("goal", 0.05)
    assert not ok and "too long" in message
    assert handle.cancelled
