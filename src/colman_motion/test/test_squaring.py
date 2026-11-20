import math

import pytest
from colman_motion.geometry import (
    cube_yaw_in_tool,
    quarter_turn,
    tool_yaw_to_square,
    yaw_of,
    yawed,
)

EE_DOWN = (1.0, 0.0, 0.0, 0.0)


def about_z(degrees):
    half = math.radians(degrees) / 2
    return (0.0, 0.0, math.sin(half), math.cos(half))


@pytest.mark.parametrize("degrees", [0, 30, -30, 90, 170])
def test_a_turned_tool_has_that_yaw(degrees):
    assert yaw_of(yawed(EE_DOWN, degrees)) == pytest.approx(
        (degrees + 180) % 360 - 180, abs=1e-6
    )


@pytest.mark.parametrize("degrees", [-90, -46, -45, 0, 44, 45, 90, 135])
def test_quarter_turn_folds_into_range(degrees):
    folded = quarter_turn(degrees)
    assert -45 <= folded < 45 and (degrees - folded) % 90 == pytest.approx(0, abs=1e-9)


def test_a_cube_turned_17_degrees_is_set_down_square_with_a_tag_at_40():
    tool_at_pick = yawed(EE_DOWN, 0)
    held = cube_yaw_in_tool(about_z(17), tool_at_pick)
    turn = tool_yaw_to_square(about_z(40), held)
    # the cube's yaw after the turn is the tool turn plus how it sits in the tool
    placed = yaw_of(yawed(EE_DOWN, turn)) + held
    assert quarter_turn(placed - 40) == pytest.approx(0, abs=1e-6)


def test_the_pick_tool_turn_does_not_change_the_placed_result():
    for pick_turn in (0, 90, 180, -90):
        held = cube_yaw_in_tool(about_z(23), yawed(EE_DOWN, pick_turn))
        turn = tool_yaw_to_square(about_z(-12), held)
        placed = yaw_of(yawed(EE_DOWN, turn)) + held
        assert quarter_turn(placed - (-12)) == pytest.approx(0, abs=1e-6)


def test_an_aligned_cube_needs_no_turn():
    held = cube_yaw_in_tool(about_z(0), yawed(EE_DOWN, 0))
    assert tool_yaw_to_square(about_z(0), held) == pytest.approx(0, abs=1e-9)
