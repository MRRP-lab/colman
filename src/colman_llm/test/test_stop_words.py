import pytest
from colman_llm.stop_words import is_stop_command


@pytest.mark.parametrize(
    "text",
    [
        "stop",
        "Stop!",
        "STOP the arm",
        "please halt",
        "Freeze.",
        "stop, stop, stop",
        "wait, stop that",
        "don't stop at the red block",  # false match, the safe direction
    ],
)
def test_stop_words_match(text):
    assert is_stop_command(text)


@pytest.mark.parametrize(
    "text",
    [
        "",
        "pick up the red block",
        "stopwatch",
        "unstoppable",
        "the bus stops here",
        "freezer",
        "halted",
    ],
)
def test_other_text_does_not_match(text):
    assert not is_stop_command(text)
