import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# servo_node sometimes waits forever for its first joint state (a race inside Servo,
# about half of all launches), while a fresh process nearly always starts fine. So it
# is restarted whenever it keeps printing the wait line.
MAX_ATTEMPTS = 6
WAIT_LINE = b"Waiting to receive robot state update"
MAX_WAIT_LINES = 3


def generate_launch_description():
    gripper_type = LaunchConfiguration("gripper_type")
    use_camera = LaunchConfiguration("use_camera")
    dry = LaunchConfiguration("dry")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="colman", package_name="colman_moveit_config")
        .robot_description(
            file_path="config/colman_moveit.urdf.xacro",
            mappings={"gripper_type": gripper_type, "use_camera": use_camera},
        )
        .robot_description_semantic(
            file_path="srdf/colman.srdf.xacro",
            mappings={"gripper_type": gripper_type, "use_camera": use_camera},
        )
        .robot_description_kinematics(file_path="config/servo_kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_yaml = os.path.join(
        get_package_share_directory("colman_moveit_config"), "config", "servo.yaml"
    )
    with open(servo_yaml) as f:
        servo_params = {"moveit_servo": yaml.safe_load(f)}

    def make_servo_node():
        return Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            output="both",
            # A servo_node stuck in that wait ignores SIGINT; don't sit through 15 s.
            sigterm_timeout="1",
            sigkill_timeout="1",
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": False},
            ],
        )

    def supervised_servo(attempt):
        node = make_servo_node()
        state = {"waits": 0, "restarting": False}

        def on_output(event):
            if attempt >= MAX_ATTEMPTS or state["restarting"]:
                return None
            state["waits"] += event.text.count(WAIT_LINE)
            if state["waits"] > MAX_WAIT_LINES:
                state["restarting"] = True
                return [
                    LogInfo(
                        msg=(
                            "servo_node stuck waiting for joint states "
                            f"(attempt {attempt}), restarting"
                        )
                    ),
                    EmitEvent(
                        event=ShutdownProcess(process_matcher=matches_action(node))
                    ),
                ]
            return None

        def on_exit(event, context):
            if state["restarting"]:
                return [supervised_servo(attempt + 1)]
            return None

        return GroupAction(
            [
                node,
                RegisterEventHandler(
                    OnProcessIO(
                        target_action=node, on_stdout=on_output, on_stderr=on_output
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(target_action=node, on_exit=on_exit)
                ),
            ]
        )

    follower = Node(
        package="colman_motion",
        executable="cube_follower",
        output="both",
        parameters=[{"dry": dry}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="vacuum"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument(
                "dry",
                default_value="false",
                description="Compute and publish the goal but never move the arm",
            ),
            supervised_servo(1),
            follower,
        ]
    )
