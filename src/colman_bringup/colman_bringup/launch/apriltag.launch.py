from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# One detector per camera. Each has its own node name (the parameter file is keyed by
# it), parameter file and image topics. The overhead detector names its tags ovh_tag_N.
DETECTORS = {
    "wrist": ("apriltag", "apriltag.yaml", "/oakd_pro"),
    "overhead": ("apriltag_overhead", "apriltag_overhead.yaml", "/overhead"),
}


def make_detector(context):
    camera = LaunchConfiguration("camera").perform(context)
    if camera not in DETECTORS:
        raise RuntimeError(f"unknown camera '{camera}', use one of {list(DETECTORS)}")
    name, default_params, prefix = DETECTORS[camera]
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = PathJoinSubstitution(
            [FindPackageShare("colman_perception"), "config", default_params]
        ).perform(context)
    return [
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name=name,
            output="screen",
            parameters=[params_file],
            respawn=True,
            respawn_delay=2.0,
            remappings=[
                ("image_rect", f"{prefix}/rgb/image"),
                ("camera_info", f"{prefix}/rgb/camera_info"),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera",
                default_value="wrist",
                description="Which camera to detect tags in: wrist or overhead",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="apriltag_ros parameter file (default: the camera's own)",
            ),
            OpaqueFunction(function=make_detector),
        ]
    )
