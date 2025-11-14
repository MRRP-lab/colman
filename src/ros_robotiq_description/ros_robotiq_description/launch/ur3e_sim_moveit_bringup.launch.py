from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch_ros.actions import SetParameter




def generate_launch_description():
    
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    set_sim_time = SetParameter(name='use_sim_time', value=True)

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
        ),
        launch_arguments={"start_external_rm": "false"}.items(),
    )

    # launch gazebo harmonic with gui enabled
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

    moveit_demo = TimerAction(
        period=8.0,
        actions=[
            # backend
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur3e_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    ])
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            # gui
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur3e_moveit_config"),
                        "launch",
                        "moveit_rviz.launch.py",
                    ])
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            ur3e_bringup,
            set_sim_time,
            gazebo,
            spawn_entity,
            clock_bridge,
            moveit_demo,
        ]
    )
