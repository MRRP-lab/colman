from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():

    # run the ros2_control nodes?
    # must be false if using gazebo
    declare_rm_arg = DeclareLaunchArgument(
        "start_external_rm",
        default_value="false",
        description="Start external controller_manager/ros2_control_node"
    )

    # path to controller YAML
    controllers_file = PathJoinSubstitution([
        FindPackageShare("ros_robotiq_description"),
        "config",
        "ur3e_robotiq_controllers.yaml",
    ])

    # path to xacro for urdf
    description_file = PathJoinSubstitution([
        FindPackageShare("ros_robotiq_description"),
        "urdf",
        "ur3e_robotiq.urdf.xacro",
    ])

    # convert xacro to urdf
    robot_description = Command(["xacro ", description_file])

    # publish tf
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
        }],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            controllers_file,
        ],
        condition=IfCondition(LaunchConfiguration("start_external_rm")),
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

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller"],
        output="screen",
    )


    # Might need this when we try to implement with hardware?
    """
    safety_limits_arg = DeclareLaunchArgument(
         "safety_limits",
         default_value="true",
         description="Enable UR safety-limits controller (only with UR driver stack).",
    )
    """


    spawn_controllers = TimerAction(
        period=0.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ],
        
    )

    # List of nodes to return
    return LaunchDescription([
        declare_rm_arg,
        robot_state_publisher,
        ros2_control_node,
        spawn_controllers,
    ])
