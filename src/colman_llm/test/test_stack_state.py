from colman_llm.stack_state import StackState


def test_first_cube_goes_on_the_base():
    state = StackState()
    assert state.next_move("white") == ("tag_2", "tag_1")
    assert state.next_move("blue") == ("tag_6", "tag_5")


def test_each_cube_goes_on_the_previous_one():
    state = StackState()
    moves = []
    for _ in range(3):
        moves.append(state.next_move("white"))
        state.confirm("white")
    assert moves == [("tag_2", "tag_1"), ("tag_3", "tag_2"), ("tag_4", "tag_3")]
    assert state.height("white") == 3


def test_full_stack_has_no_next_move():
    state = StackState()
    for _ in range(3):
        state.confirm("blue")
    assert state.next_move("blue") is None


def test_colours_are_independent():
    state = StackState()
    state.confirm("white")
    assert state.height("blue") == 0
    assert state.next_move("blue") == ("tag_6", "tag_5")


def test_spoken_names_count_the_base_as_cube_one():
    state = StackState()
    assert state.cube_name("white", "tag_2") == "white cube 2"
    assert state.cube_name("blue", "tag_8") == "blue cube 4"
