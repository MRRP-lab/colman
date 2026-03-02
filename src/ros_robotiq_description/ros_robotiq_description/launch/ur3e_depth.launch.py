from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction, OpaqueFunction, LogInfo
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter

def generate_launch_description():
    world = LaunchConfiguration("world")
    baseline_m = LaunchConfiguration("baseline_m")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")

    spawn_z = LaunchConfiguration("spawn_z")

    declared_arguments = [
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("ros_robotiq_description"),
                "worlds",
                "table_camera_world.sdf"
            ]),
            description="Gazebo world file"
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Robot spawn height"
        ),
        DeclareLaunchArgument(
            "baseline_m",
            default_value="0.075",
            description="Stereo camera baseline in meters",
        ),
        DeclareLaunchArgument(
            "enable_pointcloud",
            default_value="true",
            description="Publish point cloud from depth image",
        ),
    ]

    # This clock bridge works
    bridge_config = PathJoinSubstitution([
        FindPackageShare("ros_robotiq_description"), # Or your package name
        "config",
        "bridge_config.yaml"
    ])

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_config}], # Use param instead of arguments
        output="screen",
    )

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
    spawn_entity = TimerAction(
        period=0.0,
        actions=[
            Node(
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
        ],
    )

    # format camera info as expected by depth computation
    camera_info_republisher = Node(
        package="ros_robotiq_description",
        executable="camera_info_republisher",
        name="camera_info_republisher",
        parameters=[
            {"baseline_m": baseline_m},
            {"left_camera_info_in": "/oakd_pro/left/camera_info_raw"},
            {"right_camera_info_in": "/oakd_pro/right/camera_info_raw"},
            {"left_camera_info_out": "/oakd_pro/left/camera_info"},
            {"right_camera_info_out": "/oakd_pro/right/camera_info"},
        ],
        output="screen",
    )

    left_rectify = Node(
        package="image_proc",
        executable="rectify_node",
        name="left_rectify",
        arguments=[
            "--ros-args",
            "--log-level",
            "compressed_depth_image_transport:=fatal",
        ],
        remappings=[
            ("image", "/oakd_pro/left/image"),
            ("camera_info", "/oakd_pro/left/camera_info"),
            ("image_rect", "/oakd_pro/left/image_rect"),
        ],
    )

    right_rectify = Node(
        package="image_proc",
        executable="rectify_node",
        name="right_rectify",
        arguments=[
            "--ros-args",
            "--log-level",
            "compressed_depth_image_transport:=fatal",
        ],
        remappings=[
            ("image", "/oakd_pro/right/image"),
            ("camera_info", "/oakd_pro/right/camera_info"),
            ("image_rect", "/oakd_pro/right/image_rect"),
        ],
    )

    point_cloud = Node(
        package="stereo_image_proc",
        executable="point_cloud_node",
        name="point_cloud",
        condition=IfCondition(enable_pointcloud),
        remappings=[
            ("left/image_rect_color", "/oakd_pro/left/image_rect"),
            ("left/camera_info", "/oakd_pro/left/camera_info"),
            ("right/image_rect", "/oakd_pro/right/image_rect"),
            ("right/camera_info", "/oakd_pro/right/camera_info"),
            ("points2", "/oakd_pro/depth/points2"),
        ],
        parameters=[{
            "approximate_sync": True,
        }],
        output="screen",
    )

    moveit_demo = TimerAction(
        period=0.0,
        actions=[
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


    return LaunchDescription(
        declared_arguments +
        [
            ur3e_bringup,
            camera_info_republisher,
            left_rectify,
            right_rectify,
            point_cloud,
            gazebo,
            moveit_demo,
            gz_bridge,
            spawn_entity,
        ]
    )
