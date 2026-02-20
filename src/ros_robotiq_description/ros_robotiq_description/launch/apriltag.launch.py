from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ros_robotiq_description"), "config", "apriltag.yaml"]
                ),
                description="apriltag_ros parameter file",
            ),
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                parameters=[params_file],
                remappings=[
                    ("image_rect", "/oakd_pro/rgb/image"),
                    ("camera_info", "/oakd_pro/rgb/camera_info"),
                ],
            ),
        ]
    )
