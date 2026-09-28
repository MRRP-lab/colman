from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # run the ros2_control nodes?
    # must be false if using gazebo
    declare_rm_arg = DeclareLaunchArgument(
        "start_external_rm",
        default_value="false",
        description="Start external controller_manager/ros2_control_node",
    )

    declare_gripper_type_arg = DeclareLaunchArgument(
        "gripper_type",
        default_value="custom",
        description="Which gripper to load 'custom' or 'robotiq'",
    )

    declare_use_camera_arg = DeclareLaunchArgument(
        "use_camera", default_value="true", description="Include camera on robot"
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )

    gripper_type = LaunchConfiguration("gripper_type")
    use_camera = LaunchConfiguration("use_camera")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robotiq_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("colman_description"),
            "config",
            "robotiq_controllers.yaml",
        ]
    )

    custom_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("colman_description"),
            "config",
            "gripper_controllers.yaml",
        ]
    )

    sim_description_file = PathJoinSubstitution(
        [
            FindPackageShare("colman_gazebo"),
            "urdf",
            "colman_gazebo.xacro",
        ]
    )

    real_description_file = PathJoinSubstitution(
        [
            FindPackageShare("colman_description"),
            "urdf",
            "colman.urdf.xacro",
        ]
    )

    robot_description_sim = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                sim_description_file,
                " gripper_type:=",
                gripper_type,
                " use_camera:=",
                use_camera,
            ]
        ),
        value_type=str,
    )

    robot_description_real = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                real_description_file,
                " gripper_type:=",
                gripper_type,
                " use_camera:=",
                use_camera,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_sim = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_sim,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(use_sim_time),
    )

    robot_state_publisher_real = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_real,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=UnlessCondition(use_sim_time),
    )

    ros2_control_node_robotiq = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[robotiq_controllers_file],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("start_external_rm"),
                    "' == 'true' and '",
                    gripper_type,
                    "' == 'robotiq'",
                ]
            )
        ),
    )

    ros2_control_node_custom = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[custom_controllers_file],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("start_external_rm"),
                    "' == 'true' and '",
                    gripper_type,
                    "' == 'custom'",
                ]
            )
        ),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", gripper_type, "' == 'robotiq'"])),
    )

    custom_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", gripper_type, "' == 'custom'"])),
    )

    return LaunchDescription(
        [
            declare_rm_arg,
            declare_gripper_type_arg,
            declare_use_camera_arg,
            declare_use_sim_time_arg,
            robot_state_publisher_sim,
            robot_state_publisher_real,
            ros2_control_node_robotiq,
            ros2_control_node_custom,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            robotiq_gripper_controller_spawner,
            custom_gripper_controller_spawner,
        ]
    )
