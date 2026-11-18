"""The overhead and AprilTag launch files build their nodes from the checked-in config.

Nothing is started: the launch descriptions are generated and the nodes inspected.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

LAUNCH = Path(__file__).resolve().parents[1] / "colman_bringup" / "launch"
PERCEPTION = Path(get_package_share_directory("colman_perception")) / "config"


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH / f"{name}.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(function, **launch_configs):
    context = LaunchContext()
    for key, value in launch_configs.items():
        context.launch_configurations[key] = value
    return function(context)


def text(value):
    """A launch value as a string, whether it is plain text or substitutions."""
    if isinstance(value, str):
        return value
    return "".join(part.text for part in value)


def node_args(node):
    return [text(arg) for arg in node._Node__arguments or []]


def test_overhead_tf_builds_a_static_publisher_from_the_calibration():
    module = load_launch("overhead_tf")
    description = module.generate_launch_description()
    (action,) = description.entities
    assert isinstance(action, OpaqueFunction)
    (node,) = run(module.static_transform)
    assert isinstance(node, Node)
    args = node_args(node)
    assert len(args) == 9
    assert args[-2:] == ["base_link", "overhead_camera_optical_frame"]
    assert all(isinstance(float(a), float) for a in args[:7])
    calibration = yaml.safe_load((PERCEPTION / "overhead_camera.yaml").read_text())
    assert args == [str(a) for a in calibration["static_transform_args"]]


def test_overhead_launch_builds_camera_and_tf_nodes():
    module = load_launch("overhead")
    nodes = run(module.make_nodes)
    assert len(nodes) == 2
    args = node_args(nodes[1])
    flags = args[0::2]
    assert flags == [
        "--x",
        "--y",
        "--z",
        "--qx",
        "--qy",
        "--qz",
        "--qw",
        "--frame-id",
        "--child-frame-id",
    ]
    assert args[-1] == "overhead_camera_optical_frame"
    assert args[-3] == "base_link"
    assert module.generate_launch_description().entities


@pytest.mark.parametrize(
    "camera, node_name, ns",
    [
        ("wrist", "apriltag", "/oakd_pro"),
        ("overhead", "apriltag_overhead", "/overhead"),
    ],
)
def test_apriltag_launch_makes_a_detector_per_camera(camera, node_name, ns):
    module = load_launch("apriltag")
    (node,) = run(module.make_detector, camera=camera, params_file="")
    assert text(node._Node__node_name) == node_name
    remaps = {text(src): text(dst) for src, dst in node._Node__remappings}
    assert remaps == {
        "image_rect": f"{ns}/rgb/image",
        "camera_info": f"{ns}/rgb/camera_info",
    }


def test_apriltag_launch_rejects_an_unknown_camera():
    module = load_launch("apriltag")
    with pytest.raises(RuntimeError, match="unknown camera"):
        run(module.make_detector, camera="side", params_file="")


@pytest.mark.parametrize(
    "file, node_name, prefix",
    [
        ("apriltag.yaml", "apriltag", "tag_"),
        ("apriltag_overhead.yaml", "apriltag_overhead", "ovh_tag_"),
    ],
)
def test_detector_parameters_are_consistent(file, node_name, prefix):
    params = yaml.safe_load((PERCEPTION / file).read_text())[node_name][
        "ros__parameters"
    ]
    assert params["family"] == "36h11"
    tag = params["tag"]
    assert len(tag["ids"]) == len(tag["frames"]) == len(tag["sizes"])
    assert len(set(tag["ids"])) == len(tag["ids"])
    assert tag["frames"] == [f"{prefix}{i}" for i in tag["ids"]]
    assert all(0.01 < s < 0.5 for s in tag["sizes"])
    assert params["max_hamming"] >= 0
