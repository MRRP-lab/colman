import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def static_transform(context):
    """Publish base_link -> overhead_camera_optical_frame (the fitted overhead pose)."""
    path = os.path.join(
        get_package_share_directory("colman_perception"),
        "config",
        "overhead_camera_pose_in_base_link.yaml",
    )
    with open(path) as f:
        # [x, y, z, qx, qy, qz, qw, parent, child]
        args = yaml.safe_load(f)["static_transform_args"]
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="overhead_camera_tf",
            arguments=[str(a) for a in args],
        )
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=static_transform)])
