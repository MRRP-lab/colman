from colman_llm.tools import MOVE_ARM, POSITIONS


def test_schema_enum_matches_positions():
    enum = MOVE_ARM["input_schema"]["properties"]["position"]["enum"]
    assert enum == list(POSITIONS)
    assert len(POSITIONS) == 2
