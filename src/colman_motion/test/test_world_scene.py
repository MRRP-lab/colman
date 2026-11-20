from types import SimpleNamespace

import pytest
from colman_motion.world_scene import (
    CUBE_M,
    MARGIN_M,
    MOVED_M,
    STALE_S,
    TABLE_TOP_Z,
    Box,
    WorldSceneSync,
    column_id,
    plan_updates,
    wanted_boxes,
)


def stack(tags, x, y, height=None):
    return SimpleNamespace(
        tags=list(tags),
        height=height or len(tags),
        position=SimpleNamespace(x=x, y=y),
    )


def world(*stacks, stale=False):
    return SimpleNamespace(stacks=list(stacks), stale=stale)


class FakeScene:
    def __init__(self):
        self.added = {}
        self.removed = []

    def add_box(self, obj_id, frame, dims, pos):
        assert frame == "base_link"
        self.added[obj_id] = (dims, pos)

    def remove_object(self, obj_id):
        self.removed.append(obj_id)
        self.added.pop(obj_id, None)


class FakeLog:
    def __init__(self):
        self.warnings = []

    def warn(self, text):
        self.warnings.append(text)


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def make():
    scene, log, clock = FakeScene(), FakeLog(), Clock()
    return WorldSceneSync(scene, log, clock), scene, log, clock


# ---------------------------------------------------------------------- boxes
def test_a_single_cube_is_a_box_a_cube_wide_plus_the_margin_standing_on_the_table():
    box = Box(0.4, 0.1, 1)
    side = CUBE_M + 2 * MARGIN_M
    assert box.dims == pytest.approx([side, side, CUBE_M + MARGIN_M])
    x, y, z = box.center
    assert (x, y) == (0.4, 0.1)
    assert z == pytest.approx(TABLE_TOP_Z + box.dims[2] / 2)


def test_a_taller_stack_is_a_taller_box_resting_on_the_table():
    box = Box(0.4, 0.1, 4)
    assert box.dims[2] == pytest.approx(4 * CUBE_M + MARGIN_M)
    # its bottom is on the table surface
    assert box.center[2] - box.dims[2] / 2 == pytest.approx(TABLE_TOP_Z)


def test_a_column_is_named_for_its_top_cube():
    assert column_id(stack(["tag_1", "tag_2"], 0.3, 0.1)) == "cubes_tag_2"


def test_a_column_with_an_unidentified_top_is_named_for_where_it_stands():
    assert column_id(stack(["tag_1", "unknown"], 0.31, -0.12)) == "cubes_at_0.31_-0.12"


def test_every_column_gets_a_box_except_the_one_holding_an_excluded_tag():
    stacks = [stack(["tag_1"], 0.2, 0.0), stack(["tag_2", "tag_3"], 0.4, 0.1)]
    assert set(wanted_boxes(stacks)) == {"cubes_tag_1", "cubes_tag_3"}
    assert set(wanted_boxes(stacks, ["tag_3"])) == {"cubes_tag_1"}
    assert set(wanted_boxes(stacks, ["tag_2"])) == {"cubes_tag_1"}  # a cube under it


# -------------------------------------------------------------------- updates
def test_new_columns_are_added_and_gone_ones_removed():
    current = {"a": Box(0.2, 0.0, 1)}
    wanted = {"b": Box(0.4, 0.1, 2)}
    assert plan_updates(current, wanted) == (["a"], {"b": Box(0.4, 0.1, 2)})


def test_a_small_shift_leaves_the_box_alone():
    current = {"a": Box(0.2, 0.0, 1)}
    wanted = {"a": Box(0.2 + MOVED_M / 2, 0.0, 1)}
    assert plan_updates(current, wanted) == ([], {})


def test_a_shift_or_a_change_of_height_rebuilds_the_box():
    current = {"a": Box(0.2, 0.0, 1)}
    moved = {"a": Box(0.2 + MOVED_M * 2, 0.0, 1)}
    taller = {"a": Box(0.2, 0.0, 2)}
    assert plan_updates(current, moved) == (["a"], moved)
    assert plan_updates(current, taller) == (["a"], taller)


# ---------------------------------------------------------------- the scene sync
def test_nothing_is_added_before_a_world_state_arrives():
    sync, scene, _, _ = make()
    sync.sync()
    assert scene.added == {}


def test_the_cubes_in_the_world_state_appear_in_the_scene():
    sync, scene, _, _ = make()
    sync.on_world(
        world(stack(["tag_1"], 0.2, 0.0), stack(["tag_2", "tag_3"], 0.4, 0.1))
    )
    sync.sync()
    assert set(scene.added) == {"cubes_tag_1", "cubes_tag_3"}
    assert scene.added["cubes_tag_3"][0][2] == pytest.approx(2 * CUBE_M + MARGIN_M)


def test_a_second_sync_with_nothing_new_changes_nothing():
    sync, scene, _, _ = make()
    sync.on_world(world(stack(["tag_1"], 0.2, 0.0)))
    sync.sync()
    scene.removed.clear()
    sync.sync()
    assert scene.removed == []


def test_a_cube_lifted_off_a_stack_shortens_its_box():
    sync, scene, _, _ = make()
    sync.on_world(world(stack(["tag_2", "tag_3"], 0.4, 0.1)))
    sync.sync()
    sync.on_world(world(stack(["tag_2"], 0.4, 0.1)))
    sync.sync()
    assert "cubes_tag_3" in scene.removed and set(scene.added) == {"cubes_tag_2"}


def test_the_column_being_picked_from_is_left_out_and_comes_back_after():
    sync, scene, _, _ = make()
    sync.on_world(world(stack(["tag_1"], 0.2, 0.0), stack(["tag_2"], 0.4, 0.1)))
    sync.sync()
    sync.exclude(["tag_2"])
    assert set(scene.added) == {"cubes_tag_1"}  # at once, before the arm moves
    sync.clear_exclusions()
    assert set(scene.added) == {"cubes_tag_1", "cubes_tag_2"}


def test_excluding_nothing_is_harmless():
    sync, scene, _, _ = make()
    sync.on_world(world(stack(["tag_1"], 0.2, 0.0)))
    sync.exclude([""])
    assert set(scene.added) == {"cubes_tag_1"}


def test_a_stale_world_keeps_the_last_known_boxes_and_says_so_once():
    sync, scene, log, clock = make()
    sync.on_world(world(stack(["tag_1"], 0.2, 0.0)))
    sync.sync()
    clock.now += STALE_S + 1
    sync.sync()
    sync.sync()
    assert set(scene.added) == {"cubes_tag_1"}
    assert len(log.warnings) == 1 and "stale" in log.warnings[0]
    assert not sync.current_world()


def test_a_world_marked_stale_by_the_camera_is_not_current():
    sync, _, _, _ = make()
    assert not sync.current_world()  # nothing heard yet
    sync.on_world(world(stack(["tag_1"], 0.2, 0.0), stale=True))
    assert not sync.current_world()
    sync.on_world(world(stack(["tag_1"], 0.2, 0.0)))
    assert sync.current_world()


def test_a_fresh_world_after_a_stale_one_warns_again_next_time():
    sync, _, log, clock = make()
    sync.on_world(world(stack(["tag_1"], 0.2, 0.0)))
    clock.now += STALE_S + 1
    sync.sync()
    sync.on_world(world(stack(["tag_1"], 0.2, 0.0)))
    sync.sync()
    clock.now += STALE_S + 1
    sync.sync()
    assert len(log.warnings) == 2


def test_the_motion_code_takes_the_cube_and_table_sizes_from_the_workcell():
    from colman_description import workcell
    from colman_motion import world_scene

    assert world_scene.CUBE_M == workcell.CUBE_SIZE_M == 0.05
    assert world_scene.TABLE_TOP_Z == workcell.TABLE_TOP_Z


def test_the_held_cube_and_the_drop_height_are_a_whole_cube():
    pytest.importorskip("moveit")
    from colman_description import workcell
    from colman_motion import arm_control, motion_server

    assert arm_control.HELD_CUBE_SIDE_M == workcell.CUBE_SIZE_M
    assert motion_server.DROP_OFFSET_M == workcell.CUBE_SIZE_M
