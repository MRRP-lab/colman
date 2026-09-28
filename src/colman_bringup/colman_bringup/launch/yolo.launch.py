from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    detection = Node(
        package="colman_perception",
        executable="detection",
        name="detection",
        output="screen",
    )

    return LaunchDescription([detection])
