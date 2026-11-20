"""The executor asks whether the arm can reach before it picks anything up."""

import logging

from colman_llm.fake_arm import FakeArm
from colman_llm.world_view import Column, WorldView

LOG = logging.getLogger("test")


class Reach:
    """A reach model with a fixed set of cubes it cannot get at, by tag."""

    def __init__(self, view, no_pick=(), no_place=()):
        self.view = view
        self.no_pick = {self._xy(t) for t in no_pick}
        self.no_place = {self._xy(t) for t in no_place}
        self.asked = []

    def _xy(self, tag):
        return self.view().tag_point(tag)[:2]

    def can_pick(self, x, y, z):
        self.asked.append(("pick", x, y, z))
        return (x, y) not in self.no_pick

    def can_place_on(self, x, y, z):
        self.asked.append(("place", x, y, z))
        return (x, y) not in self.no_place


def arm_with(no_pick=(), no_place=()):
    arm = FakeArm(LOG)
    arm.reach = Reach(arm.world.view, no_pick, no_place)
    return arm, arm.executor(), arm.calls


def pap(executor, obj, dest):
    return executor.run_tool("pick_and_place", {"object": obj, "destination": dest})


def test_a_cube_out_of_reach_is_never_picked_up():
    arm, executor, calls = arm_with(no_pick=["tag_3"])
    ok, message = pap(executor, "white cube 3", "white cube 1")
    assert not ok and "white cube 3 is out of the arm's reach" in message
    assert calls == []


def test_a_destination_out_of_reach_is_refused_before_the_pick():
    arm, executor, calls = arm_with(no_place=["tag_1"])
    ok, message = pap(executor, "white cube 3", "white cube 1")
    assert (
        not ok and "can't reach the top of the stack with the white cube 1" in message
    )
    assert "haven't picked up the white cube 3" in message
    assert calls == []


def test_the_check_uses_the_height_of_the_cube_in_its_stack():
    arm, executor, _ = arm_with()
    pap(executor, "white cube 2", "white cube 1")
    arm.reach.asked.clear()
    pap(executor, "white cube 3", "white cube 2")
    kinds = {kind: z for kind, _, _, z in arm.reach.asked}
    # white cube 3 is on the table; white cube 2 is the second cube in its stack
    assert kinds["pick"] < kinds["place"]
    assert abs((kinds["place"] - kinds["pick"]) - 0.05) < 1e-9


def test_stack_cube_skips_cubes_it_cannot_reach():
    arm, executor, calls = arm_with(no_pick=["tag_2"])
    ok, _ = executor.run_tool("stack_cube", {"color": "white", "count": 3})
    assert ok
    assert calls == [
        ("pick", "tag_3"),
        ("place", "tag_1"),
        ("pick", "tag_4"),
        ("place", "tag_3"),
    ]


def test_stack_cube_says_so_when_no_free_cube_is_in_reach():
    arm, executor, calls = arm_with(no_pick=["tag_2", "tag_3", "tag_4"])
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 2})
    assert not ok and "within the arm's reach" in message and calls == []


def test_stack_cube_stops_before_picking_if_the_stack_cannot_take_another():
    arm, executor, calls = arm_with(no_place=["tag_1"])
    ok, message = executor.run_tool("stack_cube", {"color": "white", "count": 2})
    assert not ok and "can't reach the top of the stack" in message and calls == []


def test_a_cube_with_no_known_position_is_not_blocked():
    arm = FakeArm(LOG)
    view = arm.world.view()
    for column in view.columns:
        column.position = None
    arm.reach = Reach(lambda: view)  # would raise if it were asked about a position
    executor = arm.executor()
    ok, _ = pap(executor, "white cube 3", "white cube 2")
    assert ok


def test_no_reach_model_means_no_check():
    arm = FakeArm(LOG)
    assert arm.executor().run_tool("stack_cube", {"color": "white", "count": 2})[0]


def test_the_dry_run_executor_uses_the_real_model():
    from colman_llm.executor import FakeExecutor

    ok, message = FakeExecutor(LOG).run_tool(
        "stack_cube", {"color": "white", "count": 2}
    )
    assert ok, message


def test_tag_point_is_none_without_a_position():
    view = WorldView([Column(["tag_1"], 1)], "", [])
    assert view.tag_point("tag_1") is None
    assert view.tag_point("tag_2") is None
