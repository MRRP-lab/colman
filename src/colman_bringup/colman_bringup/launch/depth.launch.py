from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    baseline_m = LaunchConfiguration("baseline_m")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")

    declared_arguments = [
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

    # format camera info as expected by depth computation
    camera_info_republisher = Node(
        package="colman_perception",
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
        parameters=[
            {
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
            }
        ],
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
        parameters=[
            {
                "approximate_sync": True,
                "approximate_sync_tolerance_seconds": 1.0,
                "use_color": False,
                "use_system_default_qos": True,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            camera_info_republisher,
            left_rectify,
            right_rectify,
            disparity,
            point_cloud,
        ]
    )
