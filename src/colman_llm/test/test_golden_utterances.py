"""Golden utterances: what was said -> which tool call -> what reaches the arm.

Claude is a ScriptedClient replaying the tool_use turn recorded for each utterance, so
these run offline. They pin the whole path below the model: ClaudeClient's tool loop,
the executor's validation and the pick, place, move, stop and release calls it makes.
When the system prompt or tool schemas change, re-record the tool calls against the
real model and update CASES.
"""

import logging
import re

import pytest
from colman_llm.claude_client import ClaudeClient
from colman_llm.executor import PickAndPlaceExecutor
from colman_llm.scripted_claude import ScriptedClient

LOG = logging.getLogger("test")

# utterance, [(tool, args)], expected arm calls, spoken answer
CASES = [
    (
        "stack two blue cubes",
        [("stack_cube", {"color": "blue", "count": 2})],
        [("pick", "tag_6"), ("place", "tag_5"), ("pick", "tag_7"), ("place", "tag_6")],
        "Stacked two blue cubes.",
    ),
    (
        "build a tower of three white cubes",
        [("stack_cube", {"color": "white", "count": 3})],
        [("pick", "tag_2"), ("place", "tag_1")]
        + [
            ("pick", "tag_3"),
            ("place", "tag_2"),
            ("pick", "tag_4"),
            ("place", "tag_3"),
        ],
        "Built the tower.",
    ),
    (
        "put white cube three on blue cube one",
        [
            (
                "pick_and_place",
                {"object": "white cube 3", "destination": "blue cube 1"},
            )
        ],
        [("pick", "tag_3"), ("place", "tag_5")],
        "Done.",
    ),
    (
        "go to your home pose",
        [("go_to_pose", {"pose": "home"})],
        [("move_to", "home")],
        "At home.",
    ),
    (
        "fold up",
        [("go_to_pose", {"pose": "compact"})],
        [("move_to", "Compact")],
        "Folded.",
    ),
    ("let go of the cube", [("release_cube", {})], [("release",)], "Let go."),
    ("what are you holding", [("report_state", {})], [], "Nothing."),
    ("hold on a second", [("stop_robot", {})], [("stop",)], "Paused."),
    ("you can carry on", [("resume_robot", {})], [("resume",)], "Resuming."),
    # The model asks for something outside the allow-list: nothing reaches the arm.
    (
        "stack the purple cubes",
        [("stack_cube", {"color": "purple", "count": 2})],
        [],
        "I don't have purple cubes.",
    ),
    (
        "pick up the base of the white stack",
        [
            (
                "pick_and_place",
                {"object": "white cube 1", "destination": "blue cube 1"},
            )
        ],
        [],
        "That one cannot move.",
    ),
    ("how are you", [], [], "Fine, thanks."),
]


def make(case):
    utterance, tools, _, text = case
    script = {
        "rules": [
            {
                "when": f"^{re.escape(utterance)}$",
                "tool_calls": [{"name": n, "input": a} for n, a in tools],
                "text": text,
            }
        ]
    }
    arm = []
    executor = PickAndPlaceExecutor(
        lambda f: arm.append(("pick", f)) or (True, "ok"),
        lambda f: arm.append(("place", f)) or (True, "ok"),
        LOG,
        stop=lambda: arm.append(("stop",)) or (True, "Stopped"),
        resume=lambda: arm.append(("resume",)) or (True, "Resumed"),
        release=lambda: arm.append(("release",)) or (True, "Released"),
        move_to=lambda pose: arm.append(("move_to", pose)) or (True, "ok"),
    )
    calls = []

    def run_tool(name, args):
        calls.append((name, args))
        return executor.run_tool(name, args)

    client = ClaudeClient("claude-sonnet-5-5", LOG, client=ScriptedClient(script))
    return client, run_tool, calls, arm


@pytest.mark.parametrize("case", CASES, ids=[c[0] for c in CASES])
def test_golden_utterance(case):
    utterance, tools, arm_calls, text = case
    client, run_tool, calls, arm = make(case)
    assert client.say(utterance, run_tool) == text
    assert calls == tools
    assert arm == arm_calls


def test_unscripted_utterance_gets_the_default_reply():
    client = ClaudeClient(
        "claude-sonnet-5-5",
        LOG,
        client=ScriptedClient({"rules": [], "default": "Pardon?"}),
    )
    assert client.say("mumble", lambda n, a: (True, "ok")) == "Pardon?"


def test_rule_matching_is_case_insensitive_and_first_match_wins():
    script = {
        "rules": [
            {"when": "STOP", "tool_calls": [{"name": "stop_robot"}], "text": "one"},
            {"when": "stop", "text": "two"},
        ]
    }
    client = ClaudeClient("claude-sonnet-5-5", LOG, client=ScriptedClient(script))
    seen = []
    assert (
        client.say("please stop", lambda n, a: seen.append((n, a)) or (True, ""))
        == "one"
    )
    assert seen == [("stop_robot", {})]
