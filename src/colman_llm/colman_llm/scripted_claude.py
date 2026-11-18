"""A scripted stand-in for the Anthropic SDK client, for offline tests.

llm_node uses it instead of the real client when COLMAN_LLM_SCRIPT names a JSON file,
so the whole stack can run without ANTHROPIC_API_KEY or a network. The script is:

    {"rules": [{"when": "regex", "tool_calls": [{"name": "...", "input": {}}],
                "text": "spoken reply"}],
     "default": "spoken reply when no rule matches"}

The first rule whose regex matches the utterance (case-insensitive) wins. Its
tool_calls come back as one tool_use turn, and once the tool results are sent back its
text comes back as the final answer.
"""

import itertools
import json
import re
from types import SimpleNamespace

SCRIPT_ENV = "COLMAN_LLM_SCRIPT"


def _response(content, stop_reason):
    return SimpleNamespace(
        content=content, stop_reason=stop_reason, usage="scripted", _request_id="fake"
    )


class ScriptedClient:
    """Quacks like anthropic.Anthropic for ClaudeClient: `.messages.create(...)`."""

    def __init__(self, script):
        self._rules = script.get("rules", [])
        self._default = script.get("default", "Okay.")
        self._ids = itertools.count(1)
        self.messages = self  # ClaudeClient calls client.messages.create

    @classmethod
    def from_file(cls, path):
        with open(path) as f:
            return cls(json.load(f))

    def create(self, messages, **_kwargs):
        # The last user message that is plain text is the utterance being answered.
        text = next(
            m["content"]
            for m in reversed(messages)
            if m["role"] == "user" and isinstance(m["content"], str)
        )
        rule = next(
            (r for r in self._rules if re.search(r["when"], text, re.IGNORECASE)),
            {"text": self._default},
        )
        answered = isinstance(messages[-1]["content"], list)  # tool results are in
        calls = [] if answered else rule.get("tool_calls", [])
        if not calls:
            body = [SimpleNamespace(type="text", text=rule.get("text", ""))]
            return _response(body, "end_turn")
        blocks = [
            SimpleNamespace(
                type="tool_use",
                id=f"toolu_{next(self._ids)}",
                name=c["name"],
                input=c.get("input", {}),
            )
            for c in calls
        ]
        return _response(blocks, "tool_use")
