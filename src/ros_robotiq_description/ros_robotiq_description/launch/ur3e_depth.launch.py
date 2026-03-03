from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    RegisterEventHandler, LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world = LaunchConfiguration("world")
    baseline_m = LaunchConfiguration("baseline_m")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")
    gripper_type = LaunchConfiguration("gripper_type")

    spawn_z = LaunchConfiguration("spawn_z")

    declared_arguments = [
        DeclareLaunchArgument(
            "gripper_type",
            default_value="custom",
            description="Which gripper to load 'custom' or 'robotiq'"
        ),
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
        respawn=True,
        respawn_delay=2.0,
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
        launch_arguments={"start_external_rm": "false", "use_sim_time": "true", "gripper_type": gripper_type}.items()
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
            {"left_frame_id": "oakd_pro_left_optical_frame_wrist"},
            {"right_frame_id": "oakd_pro_left_optical_frame_wrist"},
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
        name="disparity",
        condition=IfCondition(enable_pointcloud),
        remappings=[
            ("left/image_rect", "/oakd_pro/left/image_rect"),
            ("left/camera_info", "/oakd_pro/left/camera_info"),
            ("right/image_rect", "/oakd_pro/right/image_rect"),
            ("right/camera_info", "/oakd_pro/right/camera_info"),
        ],
        parameters=[{
            "approximate_sync": True,
            "stereo_algorithm": 1,
            "min_disparity": 0,
            "texture_threshold": 20,
            "disparity_range": 128,
            "uniqueness_ratio": 30.0, 
            "speckle_size": 200,
            "speckle_range": 4,
            "disp12_max_diff": 1,
            "P1": 200.0,
            "P2": 800.0,
        }],
        output="screen",
    )

    point_cloud = Node(
        package="stereo_image_proc",
        executable="point_cloud_node",
        name="point_cloud",
        condition=IfCondition(enable_pointcloud),
        remappings=[
            ("left/image_rect_color", "/oakd_pro/left/image_rect"),
            ("left/camera_info", "/oakd_pro/left/camera_info"),
            ("right/camera_info", "/oakd_pro/right/camera_info"),
            ("points2", "/oakd_pro/depth/points2"),
        ],
        parameters=[{
            "approximate_sync": True,
            "approximate_sync_tolerance_seconds": 1.0,
            "use_color": False,
            "use_system_default_qos": True,
        }],
        output="screen",
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
            camera_info_republisher,
            left_rectify,
            right_rectify,
            disparity,
            point_cloud,
            gazebo,
            gz_bridge,
            spawn_entity,
            moveit
        ]
    )
