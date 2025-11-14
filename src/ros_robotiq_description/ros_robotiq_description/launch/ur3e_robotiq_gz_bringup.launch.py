from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction



def generate_launch_description():
    
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")

    declared_arguments = [
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Launch rviz"
        ),
        DeclareLaunchArgument(
            "world",
            default_value="empty.sdf",
            description="Gazebo world file",
        ),
    ]

    # call the template bringup for common nodes
    ur3e_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_robotiq_description"),
                "launch",
                "ur3e_robotiq_bringup.launch.py"
            ])
        )
    )

    # launch gazebo  with gui enabled
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world]
        }.items(),
    )

    # spawn the arm in gazebo
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic", "/robot_description",
                    "-name", "ur3e_robotiq",
                    "-z", "0.0"
                ],
            )
        ],
    )


    # bridge gazebo and ros clock
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("ros_robotiq_description"),
                "rviz",
                "display_config.rviz"
            ])
        ],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        declared_arguments + [
            ur3e_bringup,
            gazebo,
            spawn_entity,
            clock_bridge,
            rviz_node,
        ]
    )
