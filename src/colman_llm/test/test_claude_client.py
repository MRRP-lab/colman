import logging
from types import SimpleNamespace

import anthropic
import httpx2 as httpx
import pytest
from colman_llm.claude_client import ClaudeClient

from colman_llm import claude_client

LOG = logging.getLogger("test")


def reply(text="Hello there.", stop_reason="end_turn"):
    return SimpleNamespace(
        content=[SimpleNamespace(type="text", text=text)],
        stop_reason=stop_reason,
        usage="usage",
        _request_id="req_1",
    )


class FakeMessages:
    def __init__(self, *results):
        self.results = list(results)
        self.calls = []

    def create(self, **kwargs):
        self.calls.append(kwargs)
        result = self.results.pop(0)
        if isinstance(result, Exception):
            raise result
        return result


def make(*results):
    messages = FakeMessages(*results)
    client = ClaudeClient(
        "claude-sonnet-5-5", LOG, client=SimpleNamespace(messages=messages)
    )
    return client, messages


def status_error(cls, code):
    request = httpx.Request("POST", "https://api.anthropic.com/v1/messages")
    response = httpx.Response(code, request=request)
    return cls("error", response=response, body=None)


def test_plain_reply_and_request_settings():
    client, messages = make(reply("Hi."))
    assert client.say("hello") == "Hi."
    call = messages.calls[0]
    assert call["model"] == "claude-sonnet-5-5"
    assert call["max_tokens"] == 1024
    assert call["output_config"] == {"effort": "low"}
    for banned in ("tool_choice", "temperature", "top_p", "top_k", "thinking"):
        assert banned not in call


def test_history_keeps_whole_exchanges():
    client, messages = make(*[reply(str(i)) for i in range(7)])
    for i in range(7):
        client.say(f"q{i}")
    sent = messages.calls[-1]["messages"]
    assert len(sent) == 2 * claude_client.MAX_EXCHANGES + 1
    assert sent[0]["role"] == "user" and sent[0]["content"] == "q1"


def test_refusal_is_fixed_line_and_not_remembered():
    client, messages = make(reply("secret", "refusal"), reply("ok"))
    assert client.say("bad") == claude_client.REFUSAL_LINE
    client.say("next")
    assert len(messages.calls[1]["messages"]) == 1


@pytest.mark.parametrize(
    "error",
    [
        status_error(anthropic.RateLimitError, 429),
        status_error(anthropic.APIStatusError, 500),
        anthropic.APIConnectionError(
            request=httpx.Request("POST", "https://api.anthropic.com")
        ),
    ],
)
def test_api_errors_give_fallback_line(error):
    client, _ = make(error)
    assert client.say("hello") == claude_client.FALLBACK_LINE


def tool_reply(args=None, call_id="toolu_1"):
    if args is None:
        args = {"object": "red block", "destination": "left tray"}
    return SimpleNamespace(
        content=[
            SimpleNamespace(
                type="tool_use", id=call_id, name="pick_and_place", input=args
            )
        ],
        stop_reason="tool_use",
        usage="usage",
        _request_id="req_t",
    )


def test_tools_and_known_names_are_sent():
    client, messages = make(reply("Hi."))
    client.say("hello", lambda name, args: (True, "ok"))
    call = messages.calls[0]
    assert call["tools"][0]["name"] == "pick_and_place"
    assert "red block" in call["system"] and "left tray" in call["system"]


def test_tool_call_runs_and_result_goes_back_to_claude():
    client, messages = make(tool_reply(), reply("Done."))
    calls = []

    def run_tool(name, args):
        calls.append((name, dict(args)))
        return True, "Picked up."

    assert client.say("move the red block", run_tool) == "Done."
    assert calls == [
        ("pick_and_place", {"object": "red block", "destination": "left tray"})
    ]
    sent = messages.calls[1]["messages"]
    assert sent[-1]["role"] == "user"
    (result,) = sent[-1]["content"]
    assert result["type"] == "tool_result"
    assert result["tool_use_id"] == "toolu_1"
    assert result["is_error"] is False


def test_all_results_for_one_turn_share_one_message():
    two = SimpleNamespace(
        content=[
            SimpleNamespace(type="tool_use", id="a", name="pick_and_place", input={}),
            SimpleNamespace(type="tool_use", id="b", name="pick_and_place", input={}),
        ],
        stop_reason="tool_use",
        usage="usage",
        _request_id="req_t",
    )
    client, messages = make(two, reply("ok"))
    client.say("go", lambda n, a: (True, "ok"))
    results = messages.calls[1]["messages"][-1]["content"]
    assert [r["tool_use_id"] for r in results] == ["a", "b"]


def test_tool_failure_is_reported_as_error():
    client, messages = make(tool_reply(), reply("It failed."))
    client.say("move", lambda name, args: (False, "The pick failed."))
    (result,) = messages.calls[1]["messages"][-1]["content"]
    assert result["is_error"] is True
    assert result["content"] == "The pick failed."


def test_tool_loop_is_capped_and_not_remembered():
    rounds = claude_client.MAX_TOOL_ROUNDS
    client, messages = make(*[tool_reply() for _ in range(rounds)], reply("ok"))
    assert client.say("move", lambda n, a: (True, "ok")) == claude_client.FALLBACK_LINE
    assert len(messages.calls) == rounds
    client.say("next")
    assert len(messages.calls[-1]["messages"]) == 1
