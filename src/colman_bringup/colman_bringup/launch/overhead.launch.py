from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CALIBRATION = "overhead_camera.yaml"


def make_nodes(context):
    """The overhead camera, its fixed base_link -> camera transform and its detector."""
    share = Path(get_package_share_directory("colman_perception"))
    calibration = yaml.safe_load((share / "config" / CALIBRATION).read_text())
    # [x, y, z, qx, qy, qz, qw, parent, child], as static_transform_publisher
    # takes them.
    args = calibration["static_transform_args"]
    return [
        Node(
            package="colman_perception",
            executable="camera_publisher",
            name="camera_publisher_overhead",
            output="screen",
            parameters=[{"camera": "overhead"}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="overhead_camera_tf",
            arguments=[
                "--x",
                args[0],
                "--y",
                args[1],
                "--z",
                args[2],
                "--qx",
                args[3],
                "--qy",
                args[4],
                "--qz",
                args[5],
                "--qw",
                args[6],
                "--frame-id",
                args[7],
                "--child-frame-id",
                args[8],
            ],
        ),
    ]


def generate_launch_description():
    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("colman_bringup"), "launch", "apriltag.launch.py"]
            )
        ),
        launch_arguments={"camera": "overhead"}.items(),
        condition=IfCondition(LaunchConfiguration("apriltag")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "apriltag",
                default_value="true",
                description="Also detect tags in the overhead image (ovh_tag_N)",
            ),
            OpaqueFunction(function=make_nodes),
            detector,
        ]
    )
