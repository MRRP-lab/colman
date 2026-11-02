"""Anthropic SDK wrapper: one spoken turn in, spoken text out.

Keeps the conversation history. The history is a list of whole exchanges, so
trimming never splits a turn and thinking blocks are replayed unchanged.
"""

import anthropic

from colman_llm.tools import TOOLS

MAX_TOKENS = 1024
MAX_EXCHANGES = 5
MAX_TOOL_ROUNDS = 5

REFUSAL_LINE = "I can't help with that."
IGNORE_TOKEN = "IGNORE"
FALLBACK_LINE = "I couldn't reach my brain just now."

SYSTEM_PROMPT = """\
You control a robot arm through the provided tools.
The arm has two saved positions, "position one" and "position two".
You are overhearing a room, so most speech is not meant for you.
Only respond if the speech is clearly directed at you or the robot, or it answers a
question you just asked. Otherwise reply with exactly IGNORE and nothing else.
Only call a tool when the user clearly asks the arm to move.
Answer in one or two short spoken sentences, with no markdown, lists or emoji.
If a request is ambiguous or names something not in the tool's list, ask a question \
instead of guessing.
Report tool failures plainly.
"""


class ClaudeClient:
    def __init__(self, model, logger, client=None):
        self._model = model
        self._log = logger
        self._client = client or anthropic.Anthropic()
        self._exchanges = []  # each exchange is a list of messages

    def say(self, text, run_tool=None):
        """Send one utterance to Claude and return the text to speak.

        run_tool(name, input) -> (ok, message) runs a tool call Claude asked for.
        """
        history = [m for exchange in self._exchanges for m in exchange]
        user = {"role": "user", "content": text}
        exchange = [user]
        for _ in range(MAX_TOOL_ROUNDS):
            response = self._create(history + exchange)
            if response is None:
                return FALLBACK_LINE
            if response.stop_reason == "refusal":
                return REFUSAL_LINE

            # response.content goes into the history as-is, never edited or reordered.
            exchange.append({"role": "assistant", "content": response.content})
            calls = [b for b in response.content if b.type == "tool_use"]
            if response.stop_reason != "tool_use" or not calls or run_tool is None:
                break

            results = []
            for call in calls:
                ok, message = run_tool(call.name, call.input)
                results.append(
                    {
                        "type": "tool_result",
                        "tool_use_id": call.id,
                        "content": message,
                        "is_error": not ok,
                    }
                )
            exchange.append({"role": "user", "content": results})
        else:
            self._log.warning("Tool loop hit its round limit.")
            return FALLBACK_LINE  # history would end on a tool_result, so drop it

        reply = "".join(b.text for b in response.content if b.type == "text").strip()
        if reply == IGNORE_TOKEN:
            return ""  # not spoken to: say nothing and keep it out of the history
        self._exchanges.append(exchange)
        del self._exchanges[:-MAX_EXCHANGES]
        return reply

    def _create(self, messages):
        try:
            response = self._client.messages.create(
                model=self._model,
                max_tokens=MAX_TOKENS,
                output_config={"effort": "low"},
                system=SYSTEM_PROMPT,
                tools=TOOLS,
                messages=messages,
            )
        except anthropic.RateLimitError as exc:
            self._log.error(f"Rate limited: {exc}")
            return None
        except anthropic.APIConnectionError as exc:
            self._log.error(f"Could not connect to the API: {exc}")
            return None
        except anthropic.APIStatusError as exc:
            self._log.error(f"API error {exc.status_code}: {exc.message}")
            return None
        self._log.info(f"request_id={response._request_id} usage={response.usage}")
        return response
