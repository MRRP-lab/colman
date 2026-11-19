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


def test_a_moved_cube_is_not_stacked_again():
    state = StackState()
    state.move("tag_2", "tag_1")
    assert state.height("white") == 1
    assert state.next_move("white") == ("tag_3", "tag_2")


def test_another_colour_can_land_on_a_stack():
    state = StackState()
    state.move("tag_6", "tag_1")  # blue cube 2 onto the white base
    assert state.height("white") == 1
    assert state.next_move("white") == ("tag_2", "tag_6")
    assert state.next_move("blue") == ("tag_7", "tag_5")


def test_check_move_rules():
    state = StackState()
    assert state.check_move("tag_3", "tag_2") is None
    assert "base" in state.check_move("tag_1", "tag_2")
    assert "itself" in state.check_move("tag_3", "tag_3")
    state.move("tag_2", "tag_1")
    assert "already on" in state.check_move("tag_2", "tag_1")
    assert "already on top" in state.check_move("tag_3", "tag_1")
    state.move("tag_3", "tag_2")
    assert "white cube 3 is on top of the white cube 2" in state.check_move(
        "tag_2", "tag_5"
    )


def test_no_stack_may_go_higher_than_the_limit():
    state = StackState()
    state.move("tag_2", "tag_1")
    state.move("tag_3", "tag_2")
    state.move("tag_4", "tag_3")
    assert "3 cubes high" in state.check_move("tag_6", "tag_4")
    assert state.next_move("white") is None


def test_dropped_cube_is_loose_again():
    state = StackState()
    state.move("tag_2", "tag_1")
    state.drop("tag_2")
    assert state.height("white") == 0
    assert state.next_move("white") == ("tag_2", "tag_1")


def test_describe_lists_stacks_and_loose_cubes():
    state = StackState()
    assert state.describe().startswith("No cubes are stacked.")
    state.move("tag_2", "tag_1")
    state.move("tag_3", "tag_2")
    text = state.describe()
    assert "the white cube 1 has white cube 2 then white cube 3 on top" in text
    assert "white cube 4" in text and "blue cube 2" in text
    assert "white cube 3" not in text.split("Loose cubes on the table:")[1]
