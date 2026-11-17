"""Start everything for the voice-controlled cube stacking demo.

Arm driver, camera, AprilTag detector, motion server, speech in, LLM and speech out.
Run with `pixi run stack`. Pass `dry_run:=true` for canned LLM tool results (no arm
moves) or `driver:=false` if the arm driver is already running.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, *path, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(package) + "/" + "/".join(path)
        ),
        launch_arguments=args.items(),
    )


def generate_launch_description():
    dry_run = LaunchConfiguration("dry_run")
    driver = LaunchConfiguration("driver")
    robot_ip = LaunchConfiguration("robot_ip")

    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("driver", default_value="true"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.56.101"),
            # Arm
            Node(
                package="colman_bringup",
                executable="driver_supervisor",
                output="screen",
                parameters=[{"robot_ip": robot_ip}],
                condition=IfCondition(driver),
            ),
            # Perception
            Node(
                package="colman_perception",
                executable="camera_publisher",
                output="screen",
            ),
            include("colman_bringup", "launch", "apriltag.launch.py"),
            # Motion server, after the driver has had time to come up
            TimerAction(
                period=10.0,
                actions=[
                    include(
                        "colman_motion",
                        "launch",
                        "motion_server.launch.py",
                        gripper_type="vacuum",
                    )
                ],
            ),
            # Voice
            Node(
                package="colman_audio_output", executable="kokoro_tts", output="screen"
            ),
            Node(
                package="colman_audio_input",
                executable="speech_to_text",
                output="screen",
            ),
            Node(
                package="colman_llm",
                executable="llm_node",
                output="screen",
                parameters=[{"dry_run": dry_run}],
            ),
        ]
    )
