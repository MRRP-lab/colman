from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    script_filename = PathJoinSubstitution(
        [
            FindPackageShare("ur_client_library"),
            "resources",
            "external_control.urscript",
        ]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("colman_description"),
                        "urdf",
                        "colman.urdf.xacro",
                    ]
                ),
                " gripper_type:=",
                LaunchConfiguration("gripper_type"),
                " use_camera:=",
                LaunchConfiguration("use_camera"),
                " robot_ip:=",
                LaunchConfiguration("robot_ip"),
                " ur_type:=",
                LaunchConfiguration("ur_type"),
                " tf_prefix:=",
                LaunchConfiguration("tf_prefix"),
                " use_mock_hardware:=",
                LaunchConfiguration("use_mock_hardware"),
                " mock_sensor_commands:=",
                LaunchConfiguration("mock_sensor_commands"),
                " headless_mode:=",
                LaunchConfiguration("headless_mode"),
                " script_filename:=",
                script_filename,
                " input_recipe_filename:=",
                input_recipe_filename,
                " output_recipe_filename:=",
                output_recipe_filename,
                " reverse_ip:=",
                LaunchConfiguration("reverse_ip"),
                " script_command_port:=",
                LaunchConfiguration("script_command_port"),
                " reverse_port:=",
                LaunchConfiguration("reverse_port"),
                " script_sender_port:=",
                LaunchConfiguration("script_sender_port"),
                " trajectory_port:=",
                LaunchConfiguration("trajectory_port"),
            ]
        ),
        value_type=str,
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_type", default_value="vacuum"),
            DeclareLaunchArgument("use_camera", default_value="false"),
            DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("mock_sensor_commands", default_value="false"),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("reverse_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument("script_command_port", default_value="50004"),
            DeclareLaunchArgument("reverse_port", default_value="50001"),
            DeclareLaunchArgument("script_sender_port", default_value="50002"),
            DeclareLaunchArgument("trajectory_port", default_value="50003"),
            rsp_node,
        ]
    )
