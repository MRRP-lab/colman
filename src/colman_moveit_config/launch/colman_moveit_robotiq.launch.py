from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_camera = LaunchConfiguration("use_camera")
    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="colman", package_name="colman_moveit_config")
        .robot_description(
            file_path="config/colman_moveit.urdf.xacro",
            mappings={
                "gripper_type": "robotiq",
                "use_camera": use_camera,
            },
        )
        .robot_description_semantic(
            file_path=Path("srdf") / "colman.srdf.xacro",
            mappings={
                "gripper_type": "robotiq",
                "use_camera": use_camera,
            },
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .trajectory_execution(file_path="config/robotiq_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("colman_moveit_config"), "config", "moveit.rviz"]
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz", default_value="true", description="Launch RViz?"
            ),
            DeclareLaunchArgument(
                "use_camera",
                default_value="true",
                description="Include camera on robot",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            move_group_node,
            rviz_node,
        ]
    )
