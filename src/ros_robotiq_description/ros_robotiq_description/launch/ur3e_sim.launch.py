from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    RegisterEventHandler, LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    spawn_z = LaunchConfiguration("spawn_z")

    declared_arguments = [
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Launch rviz"
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("ros_robotiq_description"),
                "worlds",
                "empty_bullet.sdf"
            ]),
            description="Gazebo world file"
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Robot spawn height"
        ),

    ]

    # call the template bringup for common nodes
    ur3e_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_robotiq_description"),
                "launch",
                "ur3e_bringup.launch.py"
            ])
        ),
        launch_arguments={"start_external_rm": "false","use_sim_time": "true"}.items()
    )

    # launch gazebo harmonic with gui enabled
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world]
        }.items()
    )

    # spawn the arm in gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "ur3e_robotiq",
            "-x", "0.0",
            "-y", "0.0",
            "-z", spawn_z # 0.9652 for table world
        ],
    )

    # This clock bridge works
    bridge_config = PathJoinSubstitution([
        FindPackageShare("ros_robotiq_description"), # Or your package name
        "config",
        "bridge_config.yaml"
    ])

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_config}], # Use param instead of arguments
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                LogInfo(msg="Robot spawned, launching MoveIt..."),
                SetParameter(name='use_sim_time', value=True),
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
    )
    return LaunchDescription(
        declared_arguments +
        [
            ur3e_bringup,
            gazebo,
            clock_bridge,
            spawn_entity,
            moveit
        ]
    )
