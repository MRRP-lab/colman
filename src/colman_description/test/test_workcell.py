from pathlib import Path

import pytest
from colman_description.workcell import (
    CUBE_SIZE_M,
    TABLE_TOP_Z,
    TAG_SIZE_M,
    cube_top_z,
)

SRC = Path(__file__).resolve().parents[2]


def test_the_cubes_are_exactly_50_mm_and_the_tag_is_not_the_cube():
    assert CUBE_SIZE_M == 0.05
    assert TAG_SIZE_M == 0.0392
    assert TAG_SIZE_M < CUBE_SIZE_M


def test_a_cube_top_is_the_table_surface_plus_whole_cubes():
    assert cube_top_z(1) == pytest.approx(TABLE_TOP_Z + 0.05)
    assert cube_top_z(8) == pytest.approx(TABLE_TOP_Z + 0.40)
    assert cube_top_z(3) - cube_top_z(2) == pytest.approx(CUBE_SIZE_M)


def source_files(*suffixes):
    for path in SRC.rglob("*"):
        parts = set(path.parts)
        if path.suffix in suffixes and not parts & {"build", "install", "log", ".pixi"}:
            yield path


def test_the_tag_size_is_not_used_as_a_cube_size_in_any_python_file():
    # 39.2 mm is the black square of the tag. It is configured for the detector, and
    # it was once also used as a cube's height, which made the arm stop above a cube.
    offenders = [
        str(p.relative_to(SRC))
        for p in source_files(".py")
        if "0.0392" in p.read_text()
        and p.name not in ("workcell.py", "test_workcell.py")
    ]
    assert offenders == []


def test_the_tag_size_is_not_used_as_a_cube_size_in_any_sdf_model_or_world():
    offenders = [
        str(p.relative_to(SRC))
        for p in source_files(".sdf")
        if "0.0392" in p.read_text()
    ]
    assert offenders == []
