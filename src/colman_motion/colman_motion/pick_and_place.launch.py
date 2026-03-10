import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_py_yaml = os.path.join(
        get_package_share_directory("colman_moveit_config"),
        "config",
        "moveit_py.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="colman", package_name="colman_moveit_config")
        .robot_description(
            file_path="config/colman_moveit.urdf.xacro",
            mappings={"gripper_type": "vacuum", "use_camera": "false"},
        )
        .robot_description_semantic(
            file_path="srdf/colman.srdf.xacro",
            mappings={"gripper_type": "vacuum", "use_camera": "false"},
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=moveit_py_yaml)
        .to_moveit_configs()
    )

    pick_and_place_node = Node(
        name="moveit_py",
        package="colman_motion",
        executable="pick_and_place",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([pick_and_place_node])
