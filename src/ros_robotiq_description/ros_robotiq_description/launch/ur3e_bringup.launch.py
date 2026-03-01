from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # run the ros2_control nodes?
    # must be false if using gazebo
    declare_rm_arg = DeclareLaunchArgument(
        "start_external_rm",
        default_value="false",
        description="Start external controller_manager/ros2_control_node"
    )

    declare_gripper_type_arg = DeclareLaunchArgument(
        "gripper_type",
        default_value="custom",
        description="Which gripper to load 'custom' or 'robotiq'"
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock"
    )

    gripper_type = LaunchConfiguration("gripper_type")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robotiq_controllers_file = PathJoinSubstitution([
        FindPackageShare("ros_robotiq_description"), "config", "ur3e_robotiq_controllers.yaml",
    ])

    custom_controllers_file = PathJoinSubstitution([
        FindPackageShare("ros_robotiq_description"), "config", "ur3e_gripper_controllers.yaml",
    ])

    # path to xacro for urdf
    description_file = PathJoinSubstitution([
        FindPackageShare("ros_robotiq_description"),
        "urdf",
        "ur3e_robotiq.urdf.xacro",
    ])

    # convert xacro to urdf
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file,
                 " gripper_type:=", gripper_type]),
        value_type=str,
    )

    # publish tf
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    ros2_control_node_robotiq = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[robotiq_controllers_file],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("start_external_rm"), "' == 'true' and '", gripper_type, "' == 'robotiq'"
        ])),
    )

    ros2_control_node_custom = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[custom_controllers_file],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("start_external_rm"), "' == 'true' and '", gripper_type, "' == 'custom'"
        ])),
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
        condition=IfCondition(PythonExpression(["'", gripper_type, "' == 'robotiq'"]))
    )

    custom_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", gripper_type, "' == 'custom'"]))
    )

    # List of nodes to return
    return LaunchDescription([
        declare_rm_arg,
        declare_gripper_type_arg,
        declare_use_sim_time_arg,
        robot_state_publisher,
        ros2_control_node_robotiq,
        ros2_control_node_custom,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        robotiq_gripper_controller_spawner,
        custom_gripper_controller_spawner,
    ])
