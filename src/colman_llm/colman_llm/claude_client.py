z"""Anthropic SDK wrapper: one spoken turn in, spoken text out.

Keeps the conversation history. The history is a list of whole exchanges, so
trimming never splits a turn and thinking blocks are replayed unchanged.
"""

import anthropic

MAX_TOKENS = 1024
MAX_EXCHANGES = 5

REFUSAL_LINE = "I can't help with that."
FALLBACK_LINE = "I couldn't reach my brain just now."

SYSTEM_PROMPT = """\
You control a robot arm through the provided tools.
Only call a tool when the user clearly asks the arm to do something.
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

    def say(self, text):
        """Send one utterance to Claude and return the text to speak."""
        messages = [m for exchange in self._exchanges for m in exchange]
        user = {"role": "user", "content": text}
        try:
            response = self._client.messages.create(
                model=self._model,
                max_tokens=MAX_TOKENS,
                output_config={"effort": "low"},
                system=SYSTEM_PROMPT,
                messages=messages + [user],
            )
        except anthropic.RateLimitError as exc:
            self._log.error(f"Rate limited: {exc}")
            return FALLBACK_LINE
        except anthropic.APIConnectionError as exc:
            self._log.error(f"Could not connect to the API: {exc}")
            return FALLBACK_LINE
        except anthropic.APIStatusError as exc:
            self._log.error(f"API error {exc.status_code}: {exc.message}")
            return FALLBACK_LINE

        self._log.info(f"request_id={response._request_id} usage={response.usage}")

        if response.stop_reason == "refusal":
            return REFUSAL_LINE

        # response.content goes into the history as-is, never edited or reordered.
        assistant = {"role": "assistant", "content": response.content}
        self._exchanges.append([user, assistant])
        del self._exchanges[:-MAX_EXCHANGES]

        return "".join(b.text for b in response.content if b.type == "text").strip()
