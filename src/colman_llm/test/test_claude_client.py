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


@pytest.mark.parametrize("text", ["IGNORE", "ignore.", " IGNORE\n", '"IGNORE"'])
def test_ignore_reply_is_silent(text):
    client, _ = make(reply(text))
    assert client.say("what's for lunch") == ""


def test_ignore_is_not_remembered():
    client, messages = make(reply(claude_client.IGNORE_TOKEN), reply("Hi."))
    assert client.say("did you push the branch?") == ""
    assert client.say("hello robot") == "Hi."
    assert len(messages.calls[1]["messages"]) == 1


def test_ignore_only_matches_whole_reply():
    client, _ = make(reply("I will not ignore that."))
    assert client.say("hello") == "I will not ignore that."


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
    assert call["tools"][0]["name"] == "stack_cube"
    system = system_text(call)
    assert "white cube" in system and "blue cube" in system


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


def test_demo_tools_are_strict_and_sent():
    client, messages = make(reply("Hi."))
    client.say("hello", lambda n, a: (True, "ok"))
    tools = {t["name"]: t for t in messages.calls[0]["tools"]}
    assert set(tools) == {
        "stack_cube",
        "pick_and_place",
        "go_to_pose",
        "release_cube",
        "report_state",
        "stop_robot",
        "resume_robot",
    }
    for tool in tools.values():
        assert tool["strict"] is True
        assert tool["input_schema"]["additionalProperties"] is False
    for name in ("stop_robot", "release_cube", "report_state"):
        assert tools[name]["input_schema"]["properties"] == {}
    assert tools["go_to_pose"]["input_schema"]["properties"]["pose"]["enum"] == [
        "home",
        "compact",
    ]
    pick = tools["pick_and_place"]["input_schema"]["properties"]
    assert pick["object"]["enum"] == pick["destination"]["enum"]  # no special cube
    assert len(pick["object"]["enum"]) == 8
    stack = tools["stack_cube"]["input_schema"]["properties"]
    assert stack["count"]["enum"] == [2, 3, 4]
    assert tools["stack_cube"]["input_schema"]["properties"]["color"]["enum"] == [
        "white",
        "blue",
    ]


def spoken_tool_reply(words):
    response = tool_reply()
    response.content.insert(0, SimpleNamespace(type="text", text=words))
    return response


def test_words_before_a_tool_call_are_spoken_before_the_tool_runs():
    client, _ = make(spoken_tool_reply("Stacking the white cubes now."), reply("Done."))
    events = []

    def run_tool(name, args):
        events.append(("tool", name))
        return True, "ok"

    result = client.say(
        "stack the white cubes", run_tool, lambda text: events.append(("say", text))
    )
    assert events == [
        ("say", "Stacking the white cubes now."),
        ("tool", "pick_and_place"),
    ]
    assert result == "Done."  # only the words after the tool result are returned


def test_a_tool_call_with_no_words_says_nothing_first():
    client, _ = make(tool_reply(), reply("Done."))
    said = []
    client.say("go", lambda n, a: (True, "ok"), said.append)
    assert said == []


def test_an_ignore_before_a_tool_call_is_not_spoken():
    client, _ = make(spoken_tool_reply("IGNORE"), reply("Done."))
    said = []
    client.say("go", lambda n, a: (True, "ok"), said.append)
    assert said == []


def test_the_system_prompt_has_no_fixed_acknowledgement_and_no_base_cube():
    prompt = claude_client.SYSTEM_PROMPT
    assert "On it." not in prompt and "No cube is a base" in prompt
    assert "say one short sentence" in prompt


# ------------------------------------------------ the table is read fresh every request
def system_text(call):
    return " ".join(b["text"] for b in call["system"])


def test_every_request_carries_the_table_as_it_is_now():
    messages = FakeMessages(reply("a"), reply("b"))
    tables = iter(["Stacks: none.", "Stacks: red on blue."])
    client = ClaudeClient(
        "m", LOG, client=SimpleNamespace(messages=messages), world=lambda: next(tables)
    )
    client.say("one")
    client.say("two")
    assert "Stacks: none." in system_text(messages.calls[0])
    assert "Stacks: red on blue." in system_text(messages.calls[1])
    assert "Stacks: none." not in system_text(messages.calls[1])


def test_the_table_is_read_again_after_each_tool_call():
    tool = SimpleNamespace(type="tool_use", id="t1", name="report_state", input={})
    first = SimpleNamespace(
        content=[tool], stop_reason="tool_use", usage="u", _request_id="r"
    )
    messages = FakeMessages(first, reply("done"))
    tables = iter(["before the move", "after the move"])
    client = ClaudeClient(
        "m", LOG, client=SimpleNamespace(messages=messages), world=lambda: next(tables)
    )
    client.say("move it", lambda name, args: (True, "ok"))
    assert "before the move" in system_text(messages.calls[0])
    assert "after the move" in system_text(messages.calls[1])


def test_the_table_is_never_kept_in_the_history():
    messages = FakeMessages(reply("a"), reply("b"))
    client = ClaudeClient(
        "m",
        LOG,
        client=SimpleNamespace(messages=messages),
        world=lambda: "UNIQUE-TABLE-TEXT",
    )
    client.say("one")
    client.say("two")
    assert "UNIQUE-TABLE-TEXT" not in str(messages.calls[1]["messages"])


def test_no_view_is_said_plainly_and_a_failing_view_does_not_stop_it():
    messages = FakeMessages(reply("a"), reply("b"))
    client = ClaudeClient(
        "m", LOG, client=SimpleNamespace(messages=messages), world=lambda: None
    )
    client.say("one")
    assert claude_client.NO_TABLE_VIEW in system_text(messages.calls[0])

    def broken():
        raise ValueError("bad")

    client = ClaudeClient(
        "m", LOG, client=SimpleNamespace(messages=messages), world=broken
    )
    assert client.say("two") == "b"


def test_a_spoken_announcement_is_in_the_history_for_the_next_reply():
    messages = FakeMessages(reply("ok"))
    client = ClaudeClient("m", LOG, client=SimpleNamespace(messages=messages))
    client.note_spoken("I can't see the blue cube.")
    client.say("where did it go")
    sent = messages.calls[0]["messages"]
    assert sent[1] == {"role": "assistant", "content": "I can't see the blue cube."}
    assert sent[-1]["content"] == "where did it go"


def test_announcements_do_not_grow_the_history_without_limit():
    client = ClaudeClient("m", LOG, client=SimpleNamespace(messages=FakeMessages()))
    for i in range(20):
        client.note_spoken(f"change {i}")
    assert len(client._exchanges) == claude_client.MAX_EXCHANGES
