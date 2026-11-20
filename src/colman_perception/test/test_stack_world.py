import pytest
from colman_perception.stack_world import (
    ARM_CLEAR_M,
    ARM_HIDE_MAX_S,
    CARRY_MEMORY_S,
    GHOST_S,
    LOST_S,
    MOVED_XY_M,
    MOVED_Z_M,
    OUTAGE_S,
    RESTORE_S,
    SETTLE_XY_M,
    SETTLE_Z_M,
    TABLE_TOP_Z,
    UNKNOWN,
    Column,
    StackModel,
    exact_heights,
    level_height,
    level_of,
    shifted,
    should_publish,
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
        self.settled.update(snapshot.relocated)
        self.settled.update(snapshot.restored)

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
    # the stack went with it: the cube under it is carried, and nothing stays behind
    assert snap.column_of("tag_2").tags == ["tag_2", "tag_6"]
    assert [c.position for c in snap.columns] == [(0.3, -0.3)]
    snap = w.settle({"tag_6": at(0.3, -0.3, 2)}, seconds=LOST_S + 1)
    assert snap.lost == [] and snap.column_of("tag_2").tags == ["tag_2", "tag_6"]


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


# ----------------------------------------- slow covers, the arm, and stale memory
def test_a_cube_covered_by_a_cube_that_takes_seconds_to_be_seen_is_not_lost():
    w = World()
    w.settle({"tag_1": at(0.4, 0.2, 1), "tag_5": at(0.2, -0.3, 1)})
    # cube 2 is placed on cube 1, but its tag is not detected for 6 s
    snap = w.settle({"tag_5": at(0.2, -0.3, 1)}, seconds=6.0)
    assert snap.lost == []
    snap = w.settle({"tag_5": at(0.2, -0.3, 1), "tag_2": at(0.4, 0.2, 2)})
    assert snap.lost == []
    assert snap.column_of("tag_1").tags == ["tag_1", "tag_2"]


def test_a_cube_with_no_cover_arriving_is_lost_after_the_wait():
    w = World()
    w.settle({"tag_1": at(0.4, 0.2, 1), "tag_5": at(0.2, -0.3, 1)})
    w.settle({"tag_5": at(0.2, -0.3, 1)})  # the timer starts when it is first missed
    snap = w.settle({"tag_5": at(0.2, -0.3, 1)}, seconds=LOST_S + 1)
    assert snap.lost == ["tag_1"]


def test_a_covered_tag_is_remembered_for_as_long_as_its_cover_is_seen():
    w = World()
    stack = {"tag_1": at(0.4, 0.2, 1), "tag_2": at(0.4, 0.2, 2)}
    w.settle(stack)
    snap = w.settle({"tag_2": at(0.4, 0.2, 2)}, seconds=600.0)
    assert snap.lost == []
    assert snap.column_of("tag_1").tags == ["tag_1", "tag_2"]


def test_the_arm_explains_a_missing_tag_only_for_a_limited_time():
    w = World()
    w.settle({"tag_3": at(0.4, 0.2, 1), "tag_5": at(0.2, -0.3, 1)})
    arm = lambda xy: True  # noqa: E731
    w.settle({"tag_5": at(0.2, -0.3, 1)}, arm=arm)  # the timer starts here
    snap = w.settle({"tag_5": at(0.2, -0.3, 1)}, arm=arm, seconds=ARM_HIDE_MAX_S - 1)
    assert snap.lost == []
    snap = w.settle({"tag_5": at(0.2, -0.3, 1)}, arm=arm, seconds=2.0)
    assert snap.lost == ["tag_3"]


def test_a_cube_the_arm_stops_hiding_gets_the_normal_wait_afresh():
    w = World()
    w.settle({"tag_3": at(0.4, 0.2, 1), "tag_5": at(0.2, -0.3, 1)})
    arm = lambda xy: True  # noqa: E731
    w.settle({"tag_5": at(0.2, -0.3, 1)}, arm=arm, seconds=ARM_HIDE_MAX_S - 5)
    snap = w.settle({"tag_5": at(0.2, -0.3, 1)}, seconds=LOST_S - 1)
    assert snap.lost == []
    snap = w.settle({"tag_5": at(0.2, -0.3, 1)}, seconds=LOST_S)
    assert snap.lost == ["tag_3"]


def test_no_unseen_cube_stays_remembered_forever_when_nothing_explains_it():
    w = World()
    w.settle({"tag_3": at(0.306, 0.14, 1), "tag_4": at(0.329, 0.049, 1)})
    # cube 3 is moved while out of view, and cube 4 ends up near it, but not over it
    w.take("tag_3")
    w.settle({"tag_4": at(0.329, 0.049, 1)})
    snap = w.settle({"tag_4": at(0.329, 0.049, 1)}, seconds=3600.0)
    assert snap.column_of("tag_3") is None and "tag_3" in snap.missing


def test_a_ghost_left_by_a_belief_the_node_kept_is_dropped_when_the_arm_is_unknown():
    # with no joint states the node reports no arm, so nothing hides a tag
    w = World()
    w.settle({"tag_3": at(0.306, 0.14, 1), "tag_4": at(0.329, 0.049, 1)})
    w.settle({"tag_4": at(0.329, 0.049, 1)})
    snap = w.settle({"tag_4": at(0.329, 0.049, 1)}, seconds=LOST_S + 1)
    assert snap.lost == ["tag_3"]


# ----------------------------------------------- pose flips are not moves
def test_a_height_swing_from_a_pose_flip_is_not_a_move():
    still = (0.46, -0.283, 0.016)
    flipped = (0.46, -0.282, 0.039)  # z jumped 23 mm, x and y did not
    assert not shifted(flipped, still, MOVED_XY_M, MOVED_Z_M)
    assert not shifted(flipped, still, SETTLE_XY_M, SETTLE_Z_M)


def test_a_sideways_shift_is_a_move():
    assert shifted((0.46, -0.283, 0.02), (0.49, -0.283, 0.02), MOVED_XY_M, MOVED_Z_M)
    assert shifted((0.46, -0.283, 0.02), (0.463, -0.283, 0.02), SETTLE_XY_M, SETTLE_Z_M)


def test_a_cube_lifted_straight_up_is_a_move():
    assert shifted((0.4, 0.1, 0.05), (0.4, 0.1, 0.10), MOVED_XY_M, MOVED_Z_M)


# ------------------------------------------------------------ a stack is slid
def tall_stack(w, x=0.2, y=0.0):
    poses = {
        "tag_5": at(x, y, 1),
        "tag_6": at(x, y, 2),
        "tag_7": at(x, y, 3),
    }
    w.settle(poses)
    w.settle({"tag_7": poses["tag_7"]})  # the lower two are covered, remembered
    return poses


def test_a_slid_stack_takes_the_cubes_under_its_top_with_it():
    w = World()
    tall_stack(w)
    snap = w.settle({"tag_7": at(0.35, 0.05, 3)})
    column = snap.column_of("tag_7")
    assert column.tags == ["tag_5", "tag_6", "tag_7"]
    assert column.position == (0.35, 0.05)
    assert set(snap.relocated) == {"tag_5", "tag_6"} and snap.lost == []
    assert [c.height for c in snap.columns] == [3]  # nothing left at the old spot


def test_a_stack_pushed_in_several_steps_keeps_its_cubes_with_it():
    w = World()
    tall_stack(w)
    for x in (0.28, 0.36, 0.44, 0.52):
        snap = w.settle({"tag_7": at(x, 0.0, 3)})
        assert snap.column_of("tag_7").tags == ["tag_5", "tag_6", "tag_7"]
        assert snap.lost == [] and [c.height for c in snap.columns] == [3]
    snap = w.settle({"tag_7": at(0.52, 0.0, 3)}, seconds=LOST_S * 3)
    assert snap.lost == []
    assert snap.column_of("tag_5").tags == ["tag_5", "tag_6", "tag_7"]


def test_a_carried_stack_is_marked_uncertain_until_a_cube_in_it_is_seen():
    w = World()
    tall_stack(w)
    snap = w.settle({"tag_7": at(0.35, 0.05, 3)})
    assert not snap.column_of("tag_7").order_known
    # the top comes off and the cube under it is seen where the guess put it
    w.take("tag_7")
    snap = w.settle({"tag_6": at(0.35, 0.05, 2)})
    assert snap.column_of("tag_6").tags[-1] == "tag_6"
    snap = w.settle({"tag_6": at(0.35, 0.05, 2), "tag_5": at(0.5, -0.2, 1)})
    assert snap.column_of("tag_5").order_known


def test_a_top_lifted_onto_the_table_leaves_the_cubes_under_it_alone():
    w = World()
    tall_stack(w)
    snap = w.settle({"tag_7": at(0.4, 0.2, 1)})
    assert snap.relocated == {}
    assert snap.column_of("tag_7").tags == ["tag_7"]


def test_a_top_moved_onto_another_stack_of_the_same_height_is_not_a_slide():
    w = World()
    tall_stack(w)
    other = {
        "tag_1": at(0.4, 0.2, 1),
        "tag_2": at(0.4, 0.2, 2),
        "tag_3": at(0.4, 0.2, 3),
    }
    w.settle({**other, "tag_7": at(0.2, 0.0, 3)})
    snap = w.settle({"tag_3": other["tag_3"], "tag_7": at(0.4, 0.2, 3)})
    assert snap.relocated == {}


def test_nothing_is_carried_when_the_cubes_under_the_top_are_still_seen():
    w = World()
    poses = {"tag_5": at(0.2, 0.0, 1), "tag_7": at(0.2, 0.0, 2)}
    w.settle(poses)
    snap = w.settle({"tag_5": poses["tag_5"], "tag_7": at(0.35, 0.05, 2)})
    assert snap.relocated == {}


def test_a_slide_onto_a_spot_with_another_cube_is_not_carried():
    w = World()
    tall_stack(w)
    w.settle({"tag_8": at(0.35, 0.05, 1), "tag_7": at(0.2, 0.0, 3)})
    snap = w.settle({"tag_8": at(0.35, 0.05, 1), "tag_7": at(0.36, 0.05, 3)})
    assert snap.relocated == {}


def test_a_carried_cube_that_is_not_there_is_dropped_like_any_other():
    w = World()
    tall_stack(w)
    w.settle({"tag_7": at(0.35, 0.05, 3)})
    # tag_7, the cover, is taken away, and the carried cubes are never seen
    w.take("tag_7")
    w.settle({"tag_1": at(0.5, -0.2, 1)})
    w.settle({"tag_1": at(0.5, -0.2, 1)}, seconds=CARRY_MEMORY_S + 2)  # memory expires
    snap = w.settle({"tag_1": at(0.5, -0.2, 1)}, seconds=LOST_S + 1)
    assert set(snap.lost) == {"tag_5", "tag_6"}


def test_poses_given_as_lists_work_like_tuples():
    # the node passes list poses; a slide must not mix lists and tuples in a column
    w = World()
    w.settle(
        {
            "tag_5": [0.2, 0.0, 0.05],
            "tag_6": [0.2, 0.0, 0.10],
            "tag_7": [0.2, 0.0, 0.15],
        }
    )
    w.settle({"tag_7": [0.2, 0.0, 0.15]})
    snap = w.settle({"tag_7": [0.35, 0.05, 0.15], "tag_1": [0.5, -0.2, 0.05]})
    assert snap.column_of("tag_7").tags == ["tag_5", "tag_6", "tag_7"]
    assert all(isinstance(c.position, tuple) for c in snap.columns)


def test_the_snapshot_says_why_a_move_was_or_was_not_carried():
    w = World()
    tall_stack(w)
    snap = w.settle({"tag_7": at(0.4, 0.2, 1)})
    assert any("not carried" in n and "landed alone" in n for n in snap.notes)
    w = World()
    tall_stack(w)
    snap = w.settle({"tag_7": at(0.35, 0.05, 3)})
    assert any("carried tag_5, tag_6" in n and "not" not in n for n in snap.notes)


def test_a_stack_pushed_far_is_carried_once_its_top_settles_at_the_end():
    # The node's real order of events: while the top is moving it is seen (in `fresh`)
    # but not settled, so nothing is carried; when it stops and settles, the cubes under
    # it are carried to where it stopped.
    m = StackModel(ALL)
    stack = {
        "tag_5": at(0.2, 0.0, 1),
        "tag_6": at(0.2, 0.0, 2),
        "tag_7": at(0.2, 0.0, 3),
    }
    m.update(1.0, stack, {"tag_7": stack["tag_7"]})
    m.update(2.0, stack, {"tag_7": stack["tag_7"]})
    settled = {t: p for t, p in stack.items() if t != "tag_7"}  # the top moved
    for t, x in ((3.0, 0.35), (4.0, 0.5), (5.0, 0.7)):  # seen moving, not settled
        snap = m.update(t, settled, {"tag_7": at(x, 0.2, 3)})
        assert snap.relocated == {} and snap.lost == []
    settled["tag_7"] = at(0.7, 0.2, 3)  # it stopped and settled
    snap = m.update(6.0, settled, {"tag_7": at(0.7, 0.2, 3)})
    assert set(snap.relocated) == {"tag_5", "tag_6"}
    settled.update(snap.relocated)
    for t in range(7, 80):  # nothing is lost afterwards
        snap = m.update(float(t), settled, {"tag_7": at(0.7, 0.2, 3)})
        assert snap.lost == []


def test_a_top_seen_mid_air_in_a_hand_carries_nothing():
    m = StackModel(ALL)
    stack = {
        "tag_5": at(0.2, 0.0, 1),
        "tag_6": at(0.2, 0.0, 2),
        "tag_7": at(0.2, 0.0, 3),
        "tag_8": at(0.2, 0.0, 4),
    }
    m.update(1.0, stack, {"tag_8": stack["tag_8"]})
    m.update(2.0, stack, {"tag_8": stack["tag_8"]})
    settled = {t: p for t, p in stack.items() if t != "tag_8"}
    # the pair is carried through the air: tag_8 is seen 10 cm away, at level 3
    snap = m.update(3.0, settled, {"tag_8": at(0.3, 0.0, 3)})
    assert snap.relocated == {}
    # it lands two high, settles, and only then does one cube go with it
    settled["tag_8"] = at(0.45, 0.2, 2)
    snap = m.update(4.0, settled, {"tag_8": at(0.45, 0.2, 2)})
    assert set(snap.relocated) == {"tag_7"}


# --------------------------------------- a tag dropped as a cube lands on it
def drop_then_cover(w, seconds_before_cover=2.0, cover_tag="tag_8"):
    """tag_7 is dropped as moved, then a cube lands where it was."""
    w.settle({"tag_7": at(0.4, 0.2, 1), "tag_1": at(0.2, -0.3, 1)})
    w.take("tag_7")  # the node dropped its belief
    w.settle({"tag_1": at(0.2, -0.3, 1)}, seconds=0.5)  # the model notices it is gone
    w.settle({"tag_1": at(0.2, -0.3, 1)}, seconds=seconds_before_cover)
    return w.settle({"tag_1": at(0.2, -0.3, 1), cover_tag: at(0.4, 0.2, 2)})


def test_a_tag_dropped_as_a_cube_lands_on_it_is_brought_back_under_it():
    w = World()
    snap = drop_then_cover(w)
    assert set(snap.restored) == {"tag_7"}
    column = snap.column_of("tag_8")
    assert column.tags == ["tag_7", "tag_8"] and not column.order_known
    assert "tag_7" not in snap.missing


def test_a_restored_tag_is_not_lost_while_its_cover_stays():
    w = World()
    drop_then_cover(w)
    snap = w.settle({"tag_8": at(0.4, 0.2, 2)}, seconds=300.0)
    assert snap.lost == []
    assert snap.column_of("tag_7").tags == ["tag_7", "tag_8"]


def test_a_dropped_tag_is_not_restored_if_the_cover_comes_too_late():
    w = World()
    snap = drop_then_cover(w, seconds_before_cover=RESTORE_S + 2)
    assert snap.restored == {} and "tag_7" in snap.missing


def test_a_dropped_tag_seen_elsewhere_is_not_restored_under_a_new_cube():
    w = World()
    w.settle({"tag_7": at(0.4, 0.2, 1)})
    w.take("tag_7")
    w.settle({"tag_7": at(0.1, -0.4, 1)})  # it was moved: seen at its new spot
    snap = w.settle({"tag_7": at(0.1, -0.4, 1), "tag_8": at(0.4, 0.2, 2)})
    assert snap.restored == {}
    assert snap.column_of("tag_7").tags == ["tag_7"]


def test_a_dropped_tag_with_nothing_landing_on_it_stays_gone():
    w = World()
    w.settle({"tag_7": at(0.4, 0.2, 1), "tag_1": at(0.2, -0.3, 1)})
    w.take("tag_7")
    snap = w.settle({"tag_1": at(0.2, -0.3, 1)}, seconds=3.0)
    assert snap.restored == {} and "tag_7" in snap.missing


def test_a_lost_tag_is_never_restored():
    w = World()
    w.settle({"tag_7": at(0.4, 0.2, 1), "tag_1": at(0.2, -0.3, 1)})
    w.settle({"tag_1": at(0.2, -0.3, 1)})
    snap = w.settle({"tag_1": at(0.2, -0.3, 1)}, seconds=LOST_S + 1)
    assert snap.lost == ["tag_7"]
    snap = w.settle({"tag_1": at(0.2, -0.3, 1), "tag_8": at(0.4, 0.2, 2)})
    assert snap.restored == {}


# ----------------------------------- covers judged by level, partial carries, notes
def test_a_cover_one_level_up_counts_even_when_the_heights_wobble():
    w = World()
    # cube 7 reads a little high and cube 8 a little low, 2 cm apart
    w.settle({"tag_7": (0.4, 0.2, 0.16), "tag_1": at(0.2, -0.3, 1)})
    w.settle({"tag_1": at(0.2, -0.3, 1), "tag_8": (0.4, 0.2, 0.18)})
    snap = w.settle(
        {"tag_1": at(0.2, -0.3, 1), "tag_8": (0.4, 0.2, 0.18)}, seconds=LOST_S * 3
    )
    assert snap.lost == []
    assert snap.column_of("tag_7") is not None


def test_a_tag_at_the_same_level_does_not_cover_a_neighbour():
    w = World()
    w.settle({"tag_7": at(0.4, 0.2, 2), "tag_8": at(0.41, 0.2, 2)})
    w.take("tag_7")
    snap = w.settle({"tag_8": at(0.41, 0.2, 2)}, seconds=1.0)
    snap = w.settle({"tag_8": at(0.41, 0.2, 2)}, seconds=1.0)
    assert snap.column_of("tag_7") is None


def test_a_slide_carries_the_cubes_still_remembered_when_one_was_lost():
    w = World()
    tall_stack(w)
    w.take("tag_6")  # the middle cube's belief is gone
    snap = w.settle({"tag_7": at(0.35, 0.05, 3)})
    assert set(snap.relocated) == {"tag_5"}
    assert snap.column_of("tag_7").tags[0] == "tag_5"


def test_a_refused_slide_is_noted_once_not_every_frame():
    w = World()
    tall_stack(w)
    first = w.settle({"tag_7": at(0.4, 0.2, 1)})
    again = w.settle({"tag_7": at(0.4, 0.2, 1)})
    assert any("not carried" in n for n in first.notes)
    assert not any("not carried" in n for n in again.notes)


def test_the_top_two_cubes_lifted_off_together_go_with_the_top_as_a_pair():
    w = World()
    tall_stack(w)  # tag_5, tag_6, tag_7 and a fourth cube on top is tag_8 below
    w.settle({"tag_8": at(0.2, 0.0, 4), "tag_7": at(0.2, 0.0, 3)})
    w.settle({"tag_8": at(0.2, 0.0, 4)})
    snap = w.settle({"tag_8": at(0.4, 0.2, 2)})  # the pair lands two high
    assert set(snap.relocated) == {"tag_7"}
    assert snap.column_of("tag_8").tags == ["tag_7", "tag_8"]
    assert not snap.column_of("tag_8").order_known
    # what is left behind keeps its own cubes and shows where the pair came from
    left = snap.column_of("tag_5")
    assert left.tags[0] == "tag_5" and left.position == (0.2, 0.0)
    assert left.height == 2 and left.tags == ["tag_5", "tag_6"]


def test_the_lower_cubes_are_not_carried_with_a_pair():
    w = World()
    tall_stack(w)
    w.settle({"tag_8": at(0.2, 0.0, 4), "tag_7": at(0.2, 0.0, 3)})
    w.settle({"tag_8": at(0.2, 0.0, 4)})
    snap = w.settle({"tag_8": at(0.4, 0.2, 2)})
    assert "tag_5" not in snap.relocated and "tag_6" not in snap.relocated
    assert snap.lost == []


def test_a_pair_set_down_keeps_its_cube_while_the_old_stack_is_exposed():
    w = World()
    tall_stack(w)
    w.settle({"tag_8": at(0.2, 0.0, 4), "tag_7": at(0.2, 0.0, 3)})
    w.settle({"tag_8": at(0.2, 0.0, 4)})
    w.settle({"tag_8": at(0.4, 0.2, 2)})
    snap = w.settle({"tag_8": at(0.4, 0.2, 2), "tag_6": at(0.2, 0.0, 2)}, seconds=40.0)
    assert snap.lost == []
    assert snap.column_of("tag_8").tags == ["tag_7", "tag_8"]
    assert snap.column_of("tag_6").tags[-1] == "tag_6"


# ------------------------------------------------ a pair held in the hand a while
def four_stack(w):
    poses = {
        "tag_5": at(0.2, 0.0, 1),
        "tag_6": at(0.2, 0.0, 2),
        "tag_7": at(0.2, 0.0, 3),
        "tag_8": at(0.2, 0.0, 4),
    }
    w.settle(poses)
    w.settle({"tag_8": poses["tag_8"]})  # the lower three are covered, remembered


def test_a_cube_lifted_away_under_a_top_is_not_lost_while_the_top_is_in_the_hand():
    w = World()
    four_stack(w)
    w.take("tag_8")  # the top is lifted out of sight
    snap = w.settle({"tag_6": at(0.2, 0.0, 2)})  # the stack below is exposed
    snap = w.settle({"tag_6": at(0.2, 0.0, 2)}, seconds=40.0)
    assert snap.lost == []
    snap = w.settle({"tag_6": at(0.2, 0.0, 2), "tag_8": at(0.4, 0.2, 2)})
    assert set(snap.relocated) == {"tag_7"}
    assert snap.column_of("tag_8").tags == ["tag_7", "tag_8"]


def test_the_cube_under_the_top_stays_remembered_for_the_pair_after_a_long_hold():
    w = World()
    four_stack(w)
    w.take("tag_8")
    w.settle({"tag_6": at(0.2, 0.0, 2)})
    w.settle({"tag_6": at(0.2, 0.0, 2)}, seconds=40.0)
    w.settle({"tag_6": at(0.2, 0.0, 2), "tag_8": at(0.4, 0.2, 2)})
    snap = w.settle({"tag_6": at(0.2, 0.0, 2), "tag_8": at(0.4, 0.2, 2)}, seconds=300.0)
    assert snap.lost == []
    assert snap.column_of("tag_8").tags == ["tag_7", "tag_8"]


def test_a_cube_under_a_top_that_never_comes_back_is_lost_after_the_minute():
    w = World()
    four_stack(w)
    w.take("tag_8")
    w.settle({"tag_6": at(0.2, 0.0, 2)})
    lost = []
    for seconds in (
        CARRY_MEMORY_S + 1,
        LOST_S + 1,
    ):  # the memory expires, then the wait
        lost += w.settle({"tag_6": at(0.2, 0.0, 2)}, seconds=seconds).lost
    assert "tag_7" in lost


def test_a_cube_in_the_pair_that_is_seen_as_it_lands_does_not_stop_the_carry():
    w = World()
    four_stack(w)
    w.take("tag_8")
    w.settle({"tag_6": at(0.2, 0.0, 2)})
    # tag_7 is seen tilted in the pair, then the pair lands with tag_8 on top
    snap = w.settle(
        {"tag_6": at(0.2, 0.0, 2), "tag_7": at(0.4, 0.2, 1), "tag_8": at(0.4, 0.2, 2)}
    )
    assert snap.lost == []
    assert snap.column_of("tag_8").tags == ["tag_7", "tag_8"]


def test_a_cube_under_a_top_seen_mid_air_in_the_hand_is_not_dropped():
    # The top flickers into view in the hand, away from the stack, while the cube under
    # it is still remembered at the old spot and the exposed cube below is seen.
    m = StackModel(ALL)
    stack = {
        "tag_5": at(0.2, 0.0, 1),
        "tag_6": at(0.2, 0.0, 2),
        "tag_7": at(0.2, 0.0, 3),
        "tag_8": at(0.2, 0.0, 4),
    }
    m.update(1.0, stack, {"tag_8": stack["tag_8"]})
    m.update(2.0, stack, {"tag_8": stack["tag_8"]})
    beliefs = dict(stack)
    lost = []
    for t in range(3, 40):
        # tag_6 is exposed; tag_8 is seen in the hand every other frame
        fresh = {"tag_6": stack["tag_6"]}
        if t % 2:
            fresh["tag_8"] = at(0.3, 0.0, 3)
        snap = m.update(float(t), beliefs, fresh)
        lost += snap.lost
        for tag in snap.lost:
            beliefs.pop(tag, None)
    assert "tag_7" not in lost
    assert "tag_7" in beliefs


def test_a_cube_under_a_top_seen_back_on_its_stack_is_covered_as_usual():
    m = StackModel(ALL)
    stack = {"tag_5": at(0.2, 0.0, 1), "tag_6": at(0.2, 0.0, 2)}
    m.update(1.0, stack, {"tag_6": stack["tag_6"]})
    m.update(2.0, stack, {"tag_6": stack["tag_6"]})
    assert m.in_transit({"tag_6": stack["tag_6"]}) == set()
    assert m.in_transit({"tag_6": at(0.5, 0.2, 2)}) == {"tag_5"}
    assert m.in_transit({}) == {"tag_5"}


def test_a_lost_tag_says_which_rule_dropped_it():
    w = World()
    w.settle({"tag_3": at(0.4, 0.2, 1), "tag_4": at(0.4, -0.2, 1)})
    w.settle({"tag_4": at(0.4, -0.2, 1)})
    snap = w.settle({"tag_4": at(0.4, -0.2, 1)}, seconds=LOST_S + 1)
    assert "not seen and not covered" in snap.lost_why["tag_3"]


# ----------------------------------------------------- the camera is unplugged
def test_with_the_camera_down_only_a_wrist_or_held_pose_is_published():
    assert should_publish("overhead", True) and should_publish("remembered", True)
    assert should_publish("wrist", False) and should_publish("held", False)
    assert not should_publish("overhead", False)
    assert not should_publish("remembered", False)
    assert not should_publish(None, False)


def test_a_long_outage_makes_the_order_inside_a_stack_uncertain():
    w = World()
    stack = {"tag_5": at(0.2, 0.0, 1), "tag_6": at(0.2, 0.0, 2)}
    w.settle(stack)
    w.settle({"tag_6": stack["tag_6"]})  # tag_5 is covered and remembered
    assert w.settle({"tag_6": stack["tag_6"]}).column_of("tag_6").order_known
    w.see({}, seconds=0.5, alive=False)  # the camera stops
    w.see({}, seconds=OUTAGE_S + 5, alive=False)
    snap = w.settle({"tag_6": stack["tag_6"]})
    assert snap.column_of("tag_6").tags == ["tag_5", "tag_6"]
    assert not snap.column_of("tag_6").order_known
    assert any("camera was down" in n for n in snap.notes)


def test_a_short_outage_leaves_the_order_alone():
    w = World()
    stack = {"tag_5": at(0.2, 0.0, 1), "tag_6": at(0.2, 0.0, 2)}
    w.settle(stack)
    w.settle({"tag_6": stack["tag_6"]})
    w.see({}, seconds=0.5, alive=False)
    w.see({}, seconds=OUTAGE_S - 5, alive=False)
    snap = w.settle({"tag_6": stack["tag_6"]})
    assert snap.column_of("tag_6").order_known
    assert not any("camera was down" in n for n in snap.notes)


def test_a_stack_left_with_no_visible_top_is_forgotten_after_a_long_outage():
    w = World()
    stack_of_three(w)
    w.take("tag_2")
    snap = w.settle({"tag_6": at(0.2, -0.3, 1)})
    assert any(c.height == 2 and not c.top_seen for c in snap.columns)  # a ghost
    w.see({}, seconds=0.5, alive=False)
    w.see({}, seconds=OUTAGE_S + 5, alive=False)
    snap = w.settle({"tag_6": at(0.2, -0.3, 1)})
    assert all(c.top_seen for c in snap.columns)


def test_the_outage_is_timed_from_when_the_camera_stopped():
    w = World()
    w.settle({"tag_3": at(0.4, 0.2, 1)})
    w.see({}, seconds=0.5, alive=False)  # the camera stops
    w.see({}, seconds=OUTAGE_S - 2, alive=False)
    w.see({}, seconds=3.0, alive=False)  # more than 10 s since it stopped, in steps
    snap = w.settle({"tag_3": at(0.4, 0.2, 1)})
    assert any("camera was down" in n for n in snap.notes)


# ------------------------------------------------ exact heights from the levels
def test_a_level_has_an_exact_height_above_the_table_surface():
    assert level_height(1) == pytest.approx(TABLE_TOP_Z + 0.05)
    assert level_height(8) == pytest.approx(TABLE_TOP_Z + 0.40)


def test_levels_are_counted_from_the_table_surface_not_from_zero():
    for level in range(1, 9):
        assert level_of(level_height(level)) == level
        assert level_of(level_height(level) + 0.02) == level  # 2 cm of noise is fine
        assert level_of(level_height(level) - 0.02) == max(1, level)


def test_every_identified_cube_gets_the_exact_height_of_its_level():
    columns = [
        Column(["tag_1", "tag_2", "tag_3"], (0.4, 0.1), 3, True, True),
        Column(["tag_5"], (0.2, 0.0), 1, True, True),
        Column([UNKNOWN, "tag_8"], (0.3, 0.3), 2, False, True),
    ]
    exact = exact_heights(columns)
    assert exact["tag_1"] == pytest.approx(level_height(1))
    assert exact["tag_3"] == pytest.approx(level_height(3))
    assert exact["tag_5"] == pytest.approx(level_height(1))
    assert exact["tag_8"] == pytest.approx(level_height(2))  # under it, an unknown cube
    assert set(exact) == {"tag_1", "tag_2", "tag_3", "tag_5", "tag_8"}
