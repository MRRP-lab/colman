"""The executor plans from what the camera sees, not from moves it remembers."""

import logging

from colman_llm.executor import PickAndPlaceExecutor
from colman_llm.world_view import UNKNOWN, Column, WorldView

LOG = logging.getLogger("test")


def col(*tags, order_known=True, top_seen=True):
    return Column(list(tags), len(tags), order_known, top_seen)


def table(*columns, held="", missing=()):
    return WorldView(list(columns), held, list(missing))


def loose(*tags):
    return [col(t) for t in tags]


class Scene:
    """A world that the test changes between the arm's steps."""

    def __init__(self, view):
        self.view = view
        self.calls = []
        self.on_pick = lambda frame: None
        self.on_place = lambda frame: None

    def world(self):
        return self.view

    def pick(self, frame):
        self.calls.append(("pick", frame))
        self.on_pick(frame)
        return True, "ok"

    def place(self, frame):
        self.calls.append(("place", frame))
        self.on_place(frame)
        return True, "ok"

    def executor(self):
        return PickAndPlaceExecutor(
            self.pick, self.place, LOG, self.world, sleep=lambda s: None
        )


def stack(color_tags, *loose_tags):
    return table(col(*color_tags), *loose(*loose_tags))


# ----------------------------------------------------------------- planning
def test_the_target_is_the_top_of_the_tallest_stack_of_the_colour():
    view = table(col("tag_1", "tag_2", "tag_3"), col("tag_4"))
    assert view.next_move("white", 4) == ("tag_4", "tag_3", None)


def test_no_cube_is_special_the_tallest_stack_grows_wherever_it_started():
    view = table(col("tag_4", "tag_3"), col("tag_1"), col("tag_2"))
    assert view.next_move("white", 3) == ("tag_1", "tag_3", None)


def test_loose_cubes_go_first_and_a_stack_is_only_taken_apart_last():
    view = table(col("tag_1", "tag_2", "tag_3"), col("tag_4", "tag_5"))
    # The blue cube on top of tag_4 makes that stack mixed, so it is not free.
    assert view.next_move("white", 4) == (
        None,
        None,
        "There are no other free white cubes to add.",
    )
    view = table(col("tag_1", "tag_2"), col("tag_3", "tag_4"))
    assert view.next_move("white", 3) == ("tag_4", "tag_2", None)


def test_a_cube_someone_else_stacked_is_seen_as_part_of_the_stack():
    view = table(col("tag_1", "tag_3"), col("tag_2"), col("tag_4"))
    assert view.next_move("white", 4) == ("tag_2", "tag_3", None)


def test_a_covered_cube_is_not_free():
    view = table(col("tag_1"), col("tag_2", "tag_5"), col("tag_4"))
    assert view.next_move("white", 2) == ("tag_4", "tag_1", None)


def test_a_stack_that_is_tall_enough_has_no_move():
    view = table(col("tag_1", "tag_2", "tag_3", "tag_4"))
    assert view.next_move("white", 3) == (
        None,
        None,
        "There is already a stack of 4 white cubes.",
    )


def test_a_colour_with_no_visible_cubes_is_reported_not_guessed():
    view = table(col("tag_5"), col("tag_6"), missing=["tag_1", "tag_2"])
    cube, target, reason = view.next_move("white", 2)
    assert cube is None and "can't see any white cubes" in reason


def test_an_unidentified_stack_is_never_chosen():
    view = table(Column([UNKNOWN, UNKNOWN], 2, False, False), col("tag_1"))
    assert view.target_of("white")[0] == "tag_1"
    view = table(Column(["tag_1", UNKNOWN], 2, False, True), col("tag_2"))
    assert view.target_of("white")[0] == "tag_2"


def test_any_cube_can_go_on_any_cube_of_either_colour():
    view = table(col("tag_5", "tag_6"), col("tag_2"))
    assert view.check_move("tag_2", "tag_6") is None


def test_a_former_base_is_a_cube_like_any_other():
    view = table(col("tag_1"), col("tag_5"))
    assert view.check_move("tag_1", "tag_5") is None


def test_the_height_of_a_colour_is_its_tallest_pure_stack():
    view = table(col("tag_1", "tag_2", "tag_3"), col("tag_5", "tag_6"))
    assert view.height("white") == 3
    assert table(col("tag_1", "tag_5")).height("white") == 0


# ------------------------------------------------------- the world changes
def test_a_top_cube_removed_while_the_arm_picks_is_not_placed_on():
    scene = Scene(table(col("tag_1", "tag_2"), col("tag_3")))
    # While tag_3 is picked someone takes tag_2 off the stack: the top is tag_1 now.
    scene.on_pick = lambda frame: setattr(
        scene, "view", table(col("tag_1"), col("tag_2"))
    )
    scene.on_place = lambda frame: setattr(scene, "view", table(col("tag_1", "tag_3")))
    ok, _ = scene.executor().run_tool("stack_cube", {"color": "white", "count": 3})
    assert ok
    assert scene.calls == [("pick", "tag_3"), ("place", "tag_1")]


def test_a_cube_added_while_the_arm_picks_changes_the_target():
    scene = Scene(table(col("tag_1"), col("tag_3"), col("tag_4"), missing=["tag_2"]))
    scene.on_pick = lambda frame: setattr(
        scene, "view", table(col("tag_1", "tag_2"), col("tag_4"))
    )
    scene.on_place = lambda frame: setattr(
        scene, "view", table(col("tag_1", "tag_2", "tag_3"), col("tag_4"))
    )
    ok, _ = scene.executor().run_tool("stack_cube", {"color": "white", "count": 3})
    assert ok and scene.calls == [("pick", "tag_3"), ("place", "tag_2")]


def test_a_destination_covered_while_the_arm_picks_is_not_placed_on():
    scene = Scene(table(col("tag_1", "tag_2"), col("tag_3"), col("tag_4")))
    scene.on_pick = lambda frame: setattr(
        scene, "view", table(col("tag_1", "tag_2", "tag_4"))
    )
    ok, message = scene.executor().run_tool(
        "pick_and_place", {"object": "white cube 3", "destination": "white cube 2"}
    )
    assert not ok and "no longer free" in message and "holding" in message
    assert scene.calls == [("pick", "tag_3")]


def test_a_place_the_camera_never_confirms_is_reported():
    scene = Scene(table(col("tag_1"), col("tag_2")))
    ok, message = scene.executor().run_tool(
        "stack_cube", {"color": "white", "count": 2}
    )
    assert not ok and "camera does not see it" in message


def test_no_world_state_means_no_motion():
    executor = PickAndPlaceExecutor(
        lambda f: (True, "ok"), lambda f: (True, "ok"), LOG, lambda: None
    )
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 2})
    assert not ok and "can't see the table" in message
    ok, message = executor.run_tool(
        "pick_and_place", {"object": "white cube 3", "destination": "white cube 2"}
    )
    assert not ok and "can't see the table" in message
    assert "can't see the table" in executor.run_tool("report_state", {})[1]


def test_a_cube_the_camera_cannot_see_is_not_picked():
    view = table(col("tag_1"), missing=["tag_2"])
    assert "can't see the white cube 2" in view.check_move("tag_2", "tag_1")


# --------------------------------------------------------------------- report
def test_describe_says_when_the_order_is_uncertain():
    view = table(Column(["tag_1", UNKNOWN, "tag_3"], 3, False, True))
    text = view.describe()
    assert "not sure of the order" in text and "an unidentified cube" in text


def test_describe_lists_cubes_it_cannot_see():
    text = table(col("tag_1"), missing=["tag_2"]).describe()
    assert "I can't see: white cube 2" in text


def test_a_cube_in_a_column_with_no_visible_top_cannot_be_picked_or_placed_on():
    hidden = col("tag_1", top_seen=False)
    view = table(hidden, col("tag_2"), col("tag_3"))
    assert "can't see the white cube 2" not in (view.check_move("tag_2", "tag_3") or "")
    assert "can't see the top of the stack" in view.check_move("tag_2", "tag_1")
    view = table(col("tag_1"), Column(["tag_2"], 1, True, False), col("tag_3"))
    assert "can't see the white cube 2 right now" in view.check_move("tag_2", "tag_3")


def test_a_stack_with_no_visible_top_is_not_a_target():
    view = table(Column(["tag_1", "tag_2"], 2, True, False), col("tag_3"))
    target, reason = view.target_of("white")
    assert target is None and "can't see the top" in reason


def test_from_msg_carries_the_visibility_of_the_top():
    from types import SimpleNamespace

    stack = SimpleNamespace(
        tags=["tag_1", "tag_2"], height=2, order_known=True, top_seen=False
    )
    view = WorldView.from_msg(SimpleNamespace(stacks=[stack], held="", missing=[]))
    assert view.columns[0].top_seen is False
