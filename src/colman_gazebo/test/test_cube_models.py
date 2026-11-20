"""Every simulated cube is the same 50 mm as the real ones, and rests on the table."""

import json
import re
import struct
from pathlib import Path

import pytest
from colman_description.workcell import CUBE_SIZE_M

PACKAGE = Path(__file__).resolve().parents[1] / "colman_gazebo"
MODELS = ["1.5inchwoodcube", "1.5inchaprilcube", "2inchwoodcube", "2inchaprilcube"]
WORLDS = ["table_world.sdf", "table_camera_world.sdf"]
# The workstation's table top in the worlds, from where the 2-inch cubes were first
# placed (0.78159 minus half of 50.8 mm).
SIM_TABLE_TOP_Z = 0.75619


def mesh_extent_m(name):
    """The edge of the cube mesh in its file, with the scale set inside the file."""
    data = (PACKAGE / "models" / name / "meshes" / f"{name}.glb").read_bytes()
    js = json.loads(data[20 : 20 + struct.unpack("<I", data[12:16])[0]])
    spans = []
    for mesh in js["meshes"]:
        for prim in mesh["primitives"]:
            acc = js["accessors"][prim["attributes"]["POSITION"]]
            spans.append(max(h - lo for lo, h in zip(acc["min"], acc["max"])))
    scales = [n["scale"][0] for n in js.get("nodes", []) if n.get("scale")]
    return max(spans) * (scales[0] if scales else 1.0)


@pytest.mark.parametrize("name", MODELS)
def test_the_collision_box_is_a_50_mm_cube(name):
    text = (PACKAGE / "models" / name / "model.sdf").read_text()
    sizes = re.findall(r"<size>([0-9.]+) ([0-9.]+) ([0-9.]+)</size>", text)
    assert len(sizes) == 1
    assert [float(v) for v in sizes[0]] == [CUBE_SIZE_M] * 3


@pytest.mark.parametrize("name", MODELS)
def test_the_visible_cube_is_the_same_size_as_the_collision_box(name):
    text = (PACKAGE / "models" / name / "model.sdf").read_text()
    scale = float(re.search(r"<scale>([0-9.]+) ", text).group(1))
    assert mesh_extent_m(name) * scale == pytest.approx(CUBE_SIZE_M, abs=1e-4)


@pytest.mark.parametrize("world", WORLDS)
def test_every_cube_in_the_worlds_rests_on_the_table(world):
    text = (PACKAGE / "worlds" / world).read_text()
    poses = re.findall(
        r"model://(?:1\.5|2)inch(?:wood|april)cube</uri>\s*<pose>[0-9.-]+ [0-9.-]+ "
        r"([0-9.]+) ",
        text,
    )
    assert len(poses) == 4
    for z in poses:
        assert float(z) - CUBE_SIZE_M / 2 == pytest.approx(SIM_TABLE_TOP_Z, abs=1e-5)
