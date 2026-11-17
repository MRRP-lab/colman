import logging
from types import SimpleNamespace

from colman_llm.service_call import ServiceCall

LOG = logging.getLogger("test")


class Future:
    def __init__(self, value):
        self._value = value

    def result(self):
        return self._value

    def add_done_callback(self, callback):
        callback(self)


class Never:
    def add_done_callback(self, callback):
        pass


class FakeClient:
    def __init__(self, response=None, available=True, hang=False):
        self.response = response
        self.available = available
        self.hang = hang
        self.calls = 0

    def wait_for_service(self, timeout_sec):
        return self.available

    def call_async(self, request):
        self.calls += 1
        return Never() if self.hang else Future(self.response)


def make(client):
    return ServiceCall(client, "request", "stop", LOG)


def test_success():
    client = FakeClient(SimpleNamespace(success=True, message="Stopped"))
    assert make(client).call() == (True, "Stopped")


def test_failure_message_is_passed_on():
    client = FakeClient(SimpleNamespace(success=False, message="nope"))
    assert make(client).call() == (False, "nope")


def test_service_not_available():
    ok, message = make(FakeClient(available=False)).call()
    assert not ok and "not available" in message


def test_no_answer_times_out():
    ok, message = make(FakeClient(hang=True)).call(timeout_s=0.05)
    assert not ok and "did not answer" in message


def test_call_async_does_not_wait_and_skips_missing_service():
    client = FakeClient(SimpleNamespace(success=True, message="Stopped"))
    make(client).call_async()
    assert client.calls == 1
    missing = FakeClient(available=False)
    make(missing).call_async()
    assert missing.calls == 0
