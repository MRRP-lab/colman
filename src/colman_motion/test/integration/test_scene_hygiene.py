"""The planning scene returns to its baseline: the table and nothing else.

Compared after a success, a failure and a cancel. The one thing allowed to differ is
the carried cube: it may be modelled on the tip only while the vacuum holds one.
"""

import pytest


def scene(rig):
    state = rig.client.server_state()
    return state["world"], state["attached"], state["holding"]


def assert_baseline(rig, what):
    world, attached, holding = scene(rig)
    assert world == rig.baseline["world"], f"{what}: world objects changed: {world}"
    assert attached == rig.baseline["attached"], f"{what}: attached left: {attached}"
    assert not holding, f"{what}: still holding"


def test_baseline_is_just_the_table(healthy):
    assert healthy.baseline["world"] == ["table"]
    assert healthy.baseline["attached"] == []


def test_after_a_successful_move(healthy):
    assert healthy.client.move_to("Compact").wait_done().outcome == "succeeded"
    assert_baseline(healthy, "after move_to")


def test_after_a_successful_pick_and_place(healthy):
    client = healthy.client
    assert client.pick("tag_2").wait_done().outcome == "succeeded"
    world, attached, holding = scene(healthy)
    assert world == ["table"] and holding
    assert client.place("tag_1").wait_done().outcome == "succeeded"
    assert_baseline(healthy, "after place")


def test_after_a_failed_pick(healthy):
    healthy.client.hide_tags(2)
    run = healthy.client.pick("tag_2").wait_done()
    assert run.outcome == "aborted", run
    assert_baseline(healthy, "after failed pick")


def test_after_a_failed_move(healthy):
    run = healthy.client.move_pose((2.0, 0.0, 0.3)).wait_done()
    assert run.outcome == "aborted", run
    assert_baseline(healthy, "after failed move_pose")


def test_after_pick_canceled_while_moving(healthy):
    client = healthy.client
    run = client.pick("tag_2")
    assert run.wait_stage("approaching", 60)
    client.wait_arm_moving()
    run.cancel()
    run.wait_done(30)
    healthy.settle()
    assert_baseline(healthy, "after canceled pick")


def test_failed_place_leaves_the_cube_only_while_held(healthy):
    """A failed place keeps the cube on the tip; letting go must remove it."""
    client = healthy.client
    assert client.pick("tag_2").wait_done().outcome == "succeeded"
    run = client.place(position=(3.0, 0.0, 0.1)).wait_done()
    assert run.outcome == "aborted", run
    world, attached, holding = scene(healthy)
    assert holding and world == ["table"]
    # The operator lets go by hand, as the failure message tells them to.
    healthy.release()
    # The next goal of any kind plans with the arm as it is: no phantom cube.
    assert client.move_to("Compact").wait_done().outcome == "succeeded"
    assert_baseline(healthy, "after release and a move")


def test_failed_release_keeps_cube_and_holding_in_step(healthy):
    client = healthy.client
    assert client.pick("tag_2").wait_done().outcome == "succeeded"
    client.io_mode("fail")
    run = client.place("tag_1").wait_done()
    assert run.outcome == "aborted", run
    world, attached, holding = scene(healthy)
    assert holding and attached == ["held_cube"], (world, attached, holding)
    client.io_mode("ok")
    healthy.release()
    assert client.move_to("home").wait_done().outcome == "succeeded"
    assert_baseline(healthy, "after the vacuum recovered")


@pytest.mark.parametrize("cycles", [3])
def test_repeated_cycles_do_not_accumulate_objects(healthy, cycles):
    client = healthy.client
    for _ in range(cycles):
        assert client.pick("tag_2").wait_done().outcome == "succeeded"
        assert client.place("tag_1").wait_done().outcome == "succeeded"
        assert_baseline(healthy, "after a cycle")
