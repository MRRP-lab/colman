from colman_llm.world_view import UNKNOWN, Column, WorldView
from colman_llm.world_watch import (
    CONFIRM_S,
    MIN_GAP_S,
    NO_VIEW_S,
    SETTLE_S,
    Snapshot,
    WorldWatch,
    changes,
)


def name_of(tag):
    return {"tag_1": "red cube", "tag_2": "blue cube", "tag_3": "green cube"}.get(
        tag, "an unidentified cube"
    )


def view(*columns, held="", missing=()):
    return WorldView([Column(list(c), len(c)) for c in columns], held, list(missing))


APART = view(["tag_1"], ["tag_2"], ["tag_3"])


def watch_settled():
    """A watcher whose baseline is APART, at time 100."""
    w = WorldWatch(name_of)
    for t in (0, SETTLE_S + 1):
        assert w.update(APART, t, False) == []
    return w, SETTLE_S + 1


def test_nothing_is_said_while_the_table_does_not_change():
    w, t = watch_settled()
    for i in range(1, 40):
        assert w.update(APART, t + i, False) == []


def test_a_cube_that_disappears_is_announced_once_it_lasts():
    w, t = watch_settled()
    gone = view(["tag_1"], ["tag_3"], missing=["tag_2"])
    assert w.update(gone, t + 1, False) == []  # not yet: it may be a hand passing
    said = w.update(gone, t + 1 + CONFIRM_S, False)
    assert said == ["I can't see the blue cube."]
    assert w.update(gone, t + 30, False) == []  # said once


def test_a_hand_passing_over_the_table_is_not_announced():
    w, t = watch_settled()
    gone = view(["tag_1"], ["tag_3"], missing=["tag_2"])
    assert w.update(gone, t + 1, False) == []
    assert w.update(APART, t + 2, False) == []
    assert w.update(APART, t + 20, False) == []


def test_a_cube_found_again_is_announced():
    w, t = watch_settled()
    gone = view(["tag_1"], ["tag_3"], missing=["tag_2"])
    w.update(gone, t + 1, False)
    w.update(gone, t + 1 + CONFIRM_S, False)
    back = t + 1 + CONFIRM_S + MIN_GAP_S + 1
    assert w.update(APART, back, False) == []
    assert w.update(APART, back + CONFIRM_S, False) == ["I can see the blue cube."]


def test_a_cube_someone_puts_on_another_is_announced():
    w, t = watch_settled()
    stacked = view(["tag_1", "tag_2"], ["tag_3"])
    w.update(stacked, t + 1, False)
    said = w.update(stacked, t + 1 + CONFIRM_S, False)
    assert said == ["Someone put the blue cube on the red cube."]


def test_a_cube_taken_off_its_stack_is_announced():
    w = WorldWatch(name_of)
    stacked = view(["tag_1", "tag_2"], ["tag_3"])
    w.update(stacked, 0, False)
    w.update(stacked, SETTLE_S + 1, False)
    t = SETTLE_S + 1
    w.update(APART, t + 1, False)
    assert w.update(APART, t + 1 + CONFIRM_S, False) == [
        "Someone moved the blue cube off its stack."
    ]


def test_the_arms_own_work_is_never_announced():
    w, t = watch_settled()
    stacked = view(["tag_1", "tag_2"], ["tag_3"])
    for i in range(1, 30):  # busy for the whole move
        assert w.update(stacked, t + i, True) == []
    after = t + 30
    assert w.update(stacked, after, False) == []
    assert w.update(stacked, after + SETTLE_S + 1, False) == []
    assert w.update(stacked, after + 60, False) == []


def test_a_held_cube_counts_as_the_arms_work():
    w, t = watch_settled()
    held = view(["tag_1"], ["tag_3"], held="tag_2", missing=["tag_2"])
    for i in range(1, 20):
        assert w.update(held, t + i, False) == []


def test_announcements_are_spaced_out():
    w, t = watch_settled()
    one = view(["tag_1"], ["tag_2"], missing=["tag_3"])
    w.update(one, t + 1, False)
    assert w.update(one, t + 1 + CONFIRM_S, False) != []
    two = view(["tag_1"], missing=["tag_2", "tag_3"])
    w.update(two, t + 6, False)
    assert w.update(two, t + 6 + CONFIRM_S, False) == []  # too soon after the last
    assert w.update(two, t + 1 + CONFIRM_S + MIN_GAP_S, False) != []


def test_many_changes_are_summarised():
    w, t = watch_settled()
    everything = view(missing=["tag_1", "tag_2", "tag_3"])
    w.update(everything, t + 1, False)
    said = w.update(everything, t + 1 + CONFIRM_S, False)
    assert said == ["I can't see the red cube or the blue cube or the green cube."]


def test_losing_the_view_is_announced_once_and_regaining_it_too():
    w, t = watch_settled()
    assert w.update(None, t + 1, False) == []
    assert w.update(None, t + 1 + NO_VIEW_S, False) == [
        "I can't see the table right now."
    ]
    assert w.update(None, t + 60, False) == []
    assert w.update(APART, t + 61, False) == ["I can see the table again."]


def test_a_short_gap_in_the_view_is_not_announced():
    w, t = watch_settled()
    assert w.update(None, t + 1, False) == []
    assert w.update(APART, t + 2, False) == []


def test_unidentified_cubes_are_not_judged():
    snap = Snapshot.of(view([UNKNOWN, "tag_1"]))
    assert snap.under == {"tag_1": ()}
    before = Snapshot.of(view([UNKNOWN, "tag_1"]))
    after = Snapshot.of(view(["tag_1"]))
    assert changes(before, after, name_of) == []
