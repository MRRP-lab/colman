from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os

def generate_launch_description():

    moveit_py_yaml = os.path.join(
        get_package_share_directory("ros_robotiq_description"),
        "config",
        "moveit_py.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description_semantic(file_path="srdf/ur.srdf.xacro", mappings={"name": "ur3e"})
        .moveit_cpp(file_path=moveit_py_yaml)
        .to_moveit_configs()
    )

    pick_and_place_node = Node(
        name="moveit_py",
        package="ros_robotiq_description",
        executable="pick_and_place",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([pick_and_place_node])
