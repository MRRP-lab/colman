from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world = LaunchConfiguration("world")
    spawn_z = LaunchConfiguration("spawn_z")

    declared_arguments = [
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("colman_gazebo"), "worlds", "table_world.sdf"]
            ),
            description="Gazebo world file",
        ),
        DeclareLaunchArgument(
            "spawn_z", default_value="0.76619", description="Robot spawn height"
        ),
    ]

    # launch gazebo harmonic with gui enabled
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": ["-r -v 4 ", world]}.items(),
    )

    # spawn the arm in gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "colman",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            spawn_z,  # 0.76619 for table world
        ],
    )

    # This clock bridge works
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("colman_gazebo"), "config", "bridge_config.yaml"]
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(declared_arguments + [gazebo, clock_bridge, spawn_entity])
