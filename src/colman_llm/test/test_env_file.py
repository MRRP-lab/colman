from colman_llm.env_file import parse_env_file


def test_parse_env_file():
    text = "# comment\n\nA =1\nexport B= 'two'\nC=\"three\"\nD=x=y\nnot a line\n"
    assert parse_env_file(text) == {"A": "1", "B": "two", "C": "three", "D": "x=y"}
