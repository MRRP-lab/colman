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
    enable_depth_pointcloud = LaunchConfiguration("enable_depth_pointcloud")
    disparity_range = LaunchConfiguration("disparity_range")
    correlation_window_size = LaunchConfiguration("correlation_window_size")
    texture_threshold = LaunchConfiguration("texture_threshold")
    speckle_size = LaunchConfiguration("speckle_size")
    speckle_range = LaunchConfiguration("speckle_range")
    disp12_max_diff = LaunchConfiguration("disp12_max_diff")
    uniqueness_ratio = LaunchConfiguration("uniqueness_ratio")

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
            "enable_depth_pointcloud",
            default_value="true",
            description="Publish point cloud from depth image",
        ),

        # depth parameter tuning
        DeclareLaunchArgument(
            "disparity_range",
            default_value="160",
            description="Stereo disparity search range, higher sees closer objects",
        ),
        DeclareLaunchArgument(
            "correlation_window_size",
            default_value="17",
            description="Stereo block size, larger = smoother, smaller = sharper/noisier",
        ),
        DeclareLaunchArgument(
            "texture_threshold",
            default_value="2",
            description="Minimum texture to accept disparity match",
        ),
        DeclareLaunchArgument(
            "speckle_size",
            default_value="50",
            description="Remove small isolated disparity regions",
        ),
        DeclareLaunchArgument(
            "speckle_range",
            default_value="2",
            description="Disparity variation allowed within speckles",
        ),
        DeclareLaunchArgument(
            "disp12_max_diff",
            default_value="2",
            description="Left-right disparity consistency threshold",
        ),
        DeclareLaunchArgument(
            "uniqueness_ratio",
            default_value="3.0",
            description="Match uniqueness ratio, lower fills more, higher cleaner",
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
    stereo_camera_info_republisher = Node(
        package="ros_robotiq_description",
        executable="stereo_camera_info_republisher",
        name="stereo_camera_info_republisher",
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

    disparity = Node(
        package="stereo_image_proc",
        executable="disparity_node",
        name="stereo_disparity",
        parameters=[
            {"approximate_sync": True},
            {"queue_size": 50},
            {"min_disparity": 0},
            {"disparity_range": disparity_range},
            {"correlation_window_size": correlation_window_size},
            {"texture_threshold": texture_threshold},
            {"speckle_size": speckle_size},
            {"speckle_range": speckle_range},
            {"disp12_max_diff": disp12_max_diff},
            {"uniqueness_ratio": uniqueness_ratio},
        ],
        remappings=[
            ("left/image_rect", "/oakd_pro/left/image_rect"),
            ("right/image_rect", "/oakd_pro/right/image_rect"),
            ("left/camera_info", "/oakd_pro/left/camera_info"),
            ("right/camera_info", "/oakd_pro/right/camera_info"),
            ("disparity", "/oakd_pro/disparity"),
        ],
        output="screen",
    )

    # built in ros2 depth to point cloud node
    depth_point_cloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="depth_point_cloud",
        condition=IfCondition(enable_depth_pointcloud),
        remappings=[
            ("image_rect", "/oakd_pro/depth/image"),
            ("camera_info", "/oakd_pro/depth/camera_info"),
            ("depth/image_rect", "/oakd_pro/depth/image"),
            ("depth/camera_info", "/oakd_pro/depth/camera_info"),
            ("points", "/oakd_pro/depth/points2"),
        ],
        output="screen",
    )

    disparity_to_depth = Node(
        package="ros_robotiq_description",
        executable="disparity_to_depth",
        name="disparity_to_depth",
        parameters=[
            {"disparity_topic": "/oakd_pro/disparity"},
            {"depth_topic": "/oakd_pro/depth/image"},
            {"left_camera_info_topic": "/oakd_pro/left/camera_info"},
            {"depth_camera_info_topic": "/oakd_pro/depth/camera_info"},
        ],
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
            stereo_camera_info_republisher,
            left_rectify,
            right_rectify,
            disparity,
            depth_point_cloud,
            disparity_to_depth,
            gazebo,
            moveit_demo,
            gz_bridge,
            spawn_entity,

        ]
    )
