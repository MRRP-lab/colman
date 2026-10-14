from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gripper_type = LaunchConfiguration("gripper_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_camera = LaunchConfiguration("use_camera")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation clock"
        ),
        DeclareLaunchArgument(
            "gripper_type",
            default_value="custom",
            description="Which gripper to load 'custom' or 'robotiq'",
        ),
        DeclareLaunchArgument(
            "use_camera", default_value="true", description="Include camera on robot"
        ),
    ]

    moveit_custom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("colman_moveit_config"),
                    "launch",
                    "colman_moveit_custom.launch.py",
                ]
            )
        ),
        condition=IfCondition(PythonExpression(["'", gripper_type, "' == 'custom'"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_camera": use_camera,
        }.items(),
    )

    moveit_robotiq = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("colman_moveit_config"),
                    "launch",
                    "colman_moveit_robotiq.launch.py",
                ]
            )
        ),
        condition=IfCondition(PythonExpression(["'", gripper_type, "' == 'robotiq'"])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_camera": use_camera,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [moveit_custom, moveit_robotiq])
