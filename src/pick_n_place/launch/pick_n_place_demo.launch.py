from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # base will setup moveit and gazebo with arm
    base_pkg_share = get_package_share_directory('ros_robotiq_description')
    base_launch_path = os.path.join(base_pkg_share, 'launch', 'ur3e_sim.launch.py')
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path)
        #launch_arguments={}
    )


    moveit_config = MoveItConfigsBuilder("ur3e").to_dict()
    if "capabilities" in moveit_config:
        moveit_config["capabilities"] += " move_group/ExecuteTaskSolutionCapability"
    else:
        moveit_config["capabilities"] = "move_group/ExecuteTaskSolutionCapability"
    # MTC Demo node
    pick_place_demo = Node(
        package="pick_n_place",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config,
        ],
        remappings=[
            ('execute_task_solution', '/execute_task_solution')
        ]
    )

    return LaunchDescription([base_launch, pick_place_demo])
