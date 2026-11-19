from colman_perception.stack_world import (
    ARM_CLEAR_M,
    GHOST_S,
    LOST_S,
    UNKNOWN,
    StackModel,
    level_of,
)

ALL = [f"tag_{i}" for i in range(1, 9)]


def at(x, y, level):
    return (x, y, 0.05 * level)


class World:
    """Drives a StackModel with what the cameras see, one second at a time."""

    def __init__(self):
        self.model = StackModel(ALL)
        self.t = 0.0
        self.settled = {}

    def see(self, poses, held=None, arm=lambda xy: False, seconds=1.0, alive=True):
        """Poses are what is visible. Anything settled and not visible is remembered."""
        self.settled = {t: p for t, p in self.settled.items() if t not in poses}
        remembered = dict(self.settled)
        self.settled = {**remembered, **poses}
        self.t += seconds
        return self.model.update(self.t, self.settled, poses, held, arm, alive)

    def take(self, *tags):
        """A cube is lifted away: the node drops the belief of a tag that moved."""
        for tag in tags:
            self.settled.pop(tag, None)

    def forget(self, snapshot):
        for tag in snapshot.lost:
            self.settled.pop(tag, None)

    def settle(self, poses, **kw):
        snap = self.see(poses, **kw)
        self.forget(snap)
        return snap


def test_a_lone_cube_is_a_column_of_one():
    snap = World().see({"tag_3": at(0.4, 0.2, 1)})
    (column,) = snap.columns
    assert (column.tags, column.height, column.order_known) == (["tag_3"], 1, True)


def test_the_height_comes_from_the_top_tag_when_the_rest_were_never_seen():
    snap = World().see({"tag_7": at(0.4, 0.05, 4)})
    (column,) = snap.columns
    assert column.height == 4
    assert column.tags == [UNKNOWN, UNKNOWN, UNKNOWN, "tag_7"]
    assert not column.order_known


def test_levels_round_to_the_nearest_cube():
    assert [level_of(z) for z in (0.041, 0.101, 0.146, 0.20)] == [1, 2, 3, 4]


def test_a_cube_placed_on_another_covers_it_and_the_order_is_remembered():
    w = World()
    w.settle({"tag_2": at(0.4, 0.05, 1), "tag_6": at(0.4, -0.1, 1)})
    snap = w.settle({"tag_2": at(0.4, 0.05, 1), "tag_6": at(0.4, 0.05, 2)})
    assert w.t  # time moved on, the covered tag is not lost
    snap = w.settle({"tag_6": at(0.4, 0.05, 2)}, seconds=LOST_S + 1)
    column = snap.column_of("tag_2")
    assert column.tags == ["tag_2", "tag_6"] and column.order_known
    assert snap.missing == [t for t in ALL if t not in ("tag_2", "tag_6")]


def test_the_top_cube_lifted_off_uncovers_the_cube_under_it():
    w = World()
    w.settle({"tag_2": at(0.4, 0.05, 1), "tag_6": at(0.4, 0.05, 2)})
    w.settle({"tag_6": at(0.4, 0.05, 2)})
    snap = w.settle({"tag_2": at(0.4, 0.05, 1)}, held="tag_6")
    (column,) = snap.columns
    assert column.tags == ["tag_2"] and column.height == 1


def test_a_tag_that_vanishes_with_nothing_over_it_is_lost_after_a_wait():
    w = World()
    w.settle({"tag_3": at(0.4, 0.2, 1), "tag_4": at(0.4, -0.2, 1)})
    snap = w.settle({"tag_4": at(0.4, -0.2, 1)}, seconds=LOST_S - 0.5)
    assert snap.column_of("tag_3") is not None and snap.lost == []
    snap = w.settle({"tag_4": at(0.4, -0.2, 1)}, seconds=LOST_S)
    assert snap.lost == ["tag_3"]
    assert snap.column_of("tag_3") is None and "tag_3" in snap.missing


def test_a_tag_hidden_by_the_arm_is_remembered_not_lost():
    w = World()
    w.settle({"tag_3": at(0.4, 0.2, 1)})
    arm = lambda xy: True  # noqa: E731
    snap = w.settle({}, arm=arm, seconds=LOST_S * 5)
    assert snap.lost == [] and snap.column_of("tag_3") is not None
    assert not snap.column_of("tag_3").top_seen


def test_the_arm_clearance_is_a_distance_to_use_with_the_arm_line():
    assert 0.05 < ARM_CLEAR_M < 0.25


def test_a_cube_pulled_from_the_middle_of_a_stack_makes_the_order_uncertain():
    w = World()
    stack = {
        "tag_5": at(0.4, 0.05, 1),
        "tag_6": at(0.4, 0.05, 2),
        "tag_7": at(0.4, 0.05, 3),
    }
    w.settle(stack)
    w.settle({"tag_7": at(0.4, 0.05, 3)})
    # someone pulls tag_6 out; tag_7 drops one level and tag_6 is not seen again
    snap = w.settle({"tag_7": at(0.4, 0.05, 2)})
    column = snap.column_of("tag_7")
    assert column.height == 2
    assert column.tags[-1] == "tag_7" and "tag_6" not in column.tags
    assert not column.order_known
    assert "tag_6" in snap.lost


def test_a_cube_pulled_from_the_bottom_leaves_the_right_height():
    w = World()
    w.settle({"tag_5": at(0.4, 0.05, 1), "tag_6": at(0.4, 0.05, 2)})
    w.settle({"tag_6": at(0.4, 0.05, 2)})
    snap = w.settle({"tag_6": at(0.4, 0.05, 1)})
    column = snap.column_of("tag_6")
    assert column.height == 1 and column.tags == ["tag_6"]
    assert not column.order_known


def test_the_cube_removed_from_a_stack_is_found_loose_elsewhere():
    w = World()
    w.settle({"tag_2": at(0.4, 0.05, 1), "tag_6": at(0.4, 0.05, 2)})
    w.settle({"tag_6": at(0.4, 0.05, 2)})
    snap = w.settle({"tag_2": at(0.4, 0.05, 1), "tag_6": at(0.2, -0.3, 1)})
    assert snap.column_of("tag_2").tags == ["tag_2"]
    assert snap.column_of("tag_6").tags == ["tag_6"]


def test_a_stack_carried_away_by_hand_does_not_leave_ghosts():
    w = World()
    w.settle({"tag_2": at(0.4, 0.05, 1), "tag_6": at(0.4, 0.05, 2)})
    w.settle({"tag_6": at(0.4, 0.05, 2)})
    snap = w.settle({"tag_6": at(0.3, -0.3, 2)})
    assert snap.column_of("tag_2") is None or snap.column_of("tag_2").height == 1
    snap = w.settle({"tag_6": at(0.3, -0.3, 2)}, seconds=LOST_S + 1)
    assert "tag_2" in snap.lost or snap.column_of("tag_2") is None


def test_two_stacks_side_by_side_stay_separate():
    snap = World().see({"tag_2": at(0.4, 0.05, 2), "tag_6": at(0.4, 0.11, 3)})
    assert sorted(c.height for c in snap.columns) == [2, 3]


def test_a_stack_nudged_a_little_stays_one_column():
    w = World()
    w.settle({"tag_2": at(0.4, 0.05, 1), "tag_6": at(0.4, 0.05, 2)})
    snap = w.settle({"tag_6": at(0.415, 0.055, 2)})
    assert snap.column_of("tag_2").tags == ["tag_2", "tag_6"]


def test_the_held_cube_is_not_on_the_table():
    w = World()
    snap = w.settle({"tag_2": at(0.4, 0.05, 1)}, held="tag_2")
    assert snap.columns == [] and "tag_2" not in snap.missing


def test_two_tags_at_the_same_level_in_one_column_keep_the_seen_one():
    snap = World().see({"tag_2": at(0.4, 0.05, 1), "tag_3": at(0.41, 0.05, 1)})
    (column,) = snap.columns
    assert column.height == 1 and not column.order_known


def test_a_missing_cube_is_reported():
    snap = World().see({"tag_2": at(0.4, 0.05, 1)})
    assert "tag_3" in snap.missing and "tag_2" not in snap.missing


# ------------------------------------------------- the top of a stack is removed
def stack_of_three(w):
    poses = {"tag_2": at(0.4, 0.05, 3)}
    w.settle(poses)
    w.settle({**poses, "tag_6": at(0.2, -0.3, 1)})
    return poses


def test_a_stack_whose_top_was_lifted_off_stays_as_a_column_with_no_visible_top():
    w = World()
    stack_of_three(w)
    w.take("tag_2")
    snap = w.settle({"tag_6": at(0.2, -0.3, 1)})
    ghost = next(c for c in snap.columns if c.height == 2)
    assert ghost.tags == [UNKNOWN, UNKNOWN] and not ghost.top_seen
    assert not ghost.order_known
    assert ghost.position == (0.4, 0.05)


def test_the_stack_comes_back_when_the_cube_under_it_is_seen():
    w = World()
    stack_of_three(w)
    w.take("tag_2")
    w.settle({"tag_6": at(0.2, -0.3, 1)})
    snap = w.settle({"tag_6": at(0.2, -0.3, 1), "tag_7": at(0.4, 0.05, 2)})
    columns = [c for c in snap.columns if c.height == 2]
    assert len(columns) == 1 and columns[0].top_seen
    assert columns[0].tags[-1] == "tag_7"


def test_a_stack_with_no_visible_top_is_dropped_after_a_while():
    w = World()
    stack_of_three(w)
    w.take("tag_2")
    w.settle({"tag_6": at(0.2, -0.3, 1)})
    snap = w.settle({"tag_6": at(0.2, -0.3, 1)}, seconds=GHOST_S + 1)
    assert [c.height for c in snap.columns] == [1]


def test_lifting_a_single_cube_leaves_no_column():
    w = World()
    w.settle({"tag_2": at(0.4, 0.05, 1), "tag_6": at(0.2, -0.3, 1)})
    w.take("tag_2")
    snap = w.settle({"tag_6": at(0.2, -0.3, 1)})
    assert [c.tags for c in snap.columns] == [["tag_6"]]


def test_the_cube_left_under_a_removed_top_is_a_known_cube_not_a_ghost():
    w = World()
    w.settle({"tag_2": at(0.4, 0.05, 1), "tag_6": at(0.4, 0.05, 2)})
    w.settle({"tag_6": at(0.4, 0.05, 2)})
    w.take("tag_6")
    snap = w.settle({"tag_5": at(0.2, -0.3, 1)})
    column = snap.column_of("tag_2")
    assert column is not None and column.height == 1 and not column.top_seen


def test_a_ghost_is_not_made_while_the_camera_is_down():
    w = World()
    stack_of_three(w)
    w.take("tag_2")
    snap = w.see({"tag_6": at(0.2, -0.3, 1)}, alive=False)
    assert [c.height for c in snap.columns] == [1]


# --------------------------------------------------------- camera stopped, stale
def test_nothing_is_lost_while_the_camera_is_down_and_the_snapshot_is_stale():
    w = World()
    w.settle({"tag_3": at(0.4, 0.2, 1), "tag_4": at(0.4, -0.2, 1)})
    snap = w.see({}, seconds=LOST_S * 10, alive=False)
    assert snap.stale and snap.lost == []
    assert snap.column_of("tag_3") is not None and snap.column_of("tag_4") is not None


def test_tags_vanishing_after_the_camera_comes_back_get_the_full_wait():
    w = World()
    w.settle({"tag_3": at(0.4, 0.2, 1)})
    w.see({}, seconds=LOST_S * 10, alive=False)
    snap = w.see({}, seconds=LOST_S - 1)
    assert not snap.stale and snap.lost == []
    snap = w.see({}, seconds=LOST_S)
    assert snap.lost == ["tag_3"]


def test_a_short_dropout_does_not_lose_anything():
    assert LOST_S >= 5.0
    w = World()
    w.settle({"tag_3": at(0.4, 0.2, 1), "tag_4": at(0.4, -0.2, 1)})
    snap = w.settle({}, seconds=3.0)
    assert snap.lost == []
    snap = w.settle({"tag_3": at(0.4, 0.2, 1), "tag_4": at(0.4, -0.2, 1)})
    assert snap.lost == [] and len(snap.columns) == 2
