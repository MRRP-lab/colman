"""Motion-level integration test: the real motion server against mock UR hardware.

Starts the UR driver in mock-hardware mode, the motion server (MoveIt, planning, the
vacuum IO) and a fake AprilTag publisher, then sends real /pick and /place goals. No
Gazebo and no display are needed. It runs with the rest of `colcon test` in CI, and on
its own with `pixi run test-integration`.

Everything runs in its own ROS domain with DDS limited to localhost, so it cannot reach
a real arm or another developer's stack on the same network.
"""

import os
import sys
import time
from pathlib import Path

import pytest
from harness import Stack, isolated_env

STARTUP_TIMEOUT_S = 180
GOAL_TIMEOUT_S = 240
HERE = Path(__file__).parent


@pytest.fixture(scope="module")
def stack(tmp_path_factory):
    from ament_index_python.packages import get_package_prefix

    env = isolated_env()
    os.environ.update(
        ROS_DOMAIN_ID=env["ROS_DOMAIN_ID"], ROS_LOCALHOST_ONLY="1"
    )  # this process's own rclpy joins the same isolated domain
    log_dir = tmp_path_factory.mktemp("integration_logs")
    stack = Stack(env, log_dir)
    rsp = Path(get_package_prefix("colman_bringup")) / "share/colman_bringup/launch"
    try:
        stack.start(
            "mock_driver",
            [
                "ros2", "launch", "ur_robot_driver", "ur_control.launch.py",
                "ur_type:=ur3e", "robot_ip:=0.0.0.0", "use_mock_hardware:=true",
                "launch_dashboard_client:=false", "launch_rviz:=false",
                f"description_launchfile:={rsp / 'rsp.launch.py'}",
            ],
        )  # fmt: skip
        stack.start("fake_tags", [sys.executable, str(HERE / "fake_tags.py")])
        time.sleep(20)  # the motion server waits for the arm to be ready
        stack.start(
            "motion_server",
            [
                "ros2", "launch", "colman_motion", "motion_server.launch.py",
                "gripper_type:=vacuum",
            ],
        )  # fmt: skip
        yield stack
    finally:
        stack.stop()
        print(f"integration logs: {log_dir}")


@pytest.fixture(scope="module")
def node(stack):
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("integration_test_client")
    # The motion server starts out assuming it holds a cube and lets go once it is
    # running. Wait for that so the first pick does not race it.
    from rclpy.action import ActionClient

    from colman_interfaces.action import Pick

    assert ActionClient(node, Pick, "/pick").wait_for_server(
        timeout_sec=STARTUP_TIMEOUT_S
    ), "motion server did not start"
    release(node)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def send_goal(node, action_type, name, goal):
    """Send a goal, wait for the result, return (success, message)."""
    import rclpy
    from rclpy.action import ActionClient

    client = ActionClient(node, action_type, name)
    assert client.wait_for_server(timeout_sec=STARTUP_TIMEOUT_S), f"no {name} server"
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30)
    handle = future.result()
    assert handle is not None and handle.accepted, f"{name} goal was not accepted"
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=GOAL_TIMEOUT_S)
    result = result_future.result()
    assert result is not None, f"{name} did not finish"
    return result.result.success, result.result.message


def release(node):
    import rclpy
    from std_srvs.srv import Trigger

    client = node.create_client(Trigger, "/release")
    assert client.wait_for_service(timeout_sec=30)
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)


def pick(node, frame):
    from colman_interfaces.action import Pick

    return send_goal(node, Pick, "/pick", Pick.Goal(frame=frame))


def place(node, location):
    from colman_interfaces.action import Place

    return send_goal(node, Place, "/place", Place.Goal(location=location))


def test_pick_loose_cube(node):
    try:
        success, message = pick(node, "tag_2")
        assert success, message
    finally:
        release(node)


def test_pick_then_place_on_a_tag(node):
    try:
        success, message = pick(node, "tag_2")
        assert success, message
        success, message = place(node, "tag_1")
        assert success, message
    finally:
        release(node)
