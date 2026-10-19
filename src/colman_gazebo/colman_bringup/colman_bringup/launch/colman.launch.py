from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            "sim", default_value="true", description="Launch Gazebo simulation"
        ),
        DeclareLaunchArgument(
            "moveit", default_value="true", description="Launch MoveIt"
        ),
        DeclareLaunchArgument(
            "gripper_type",
            default_value="custom",
            description="Which gripper to load 'custom' or 'robotiq'",
        ),
        DeclareLaunchArgument(
            "use_camera", default_value="true", description="Include camera on robot"
        ),
        DeclareLaunchArgument(
            "depth", default_value="false", description="Launch stereo depth pipeline"
        ),
        DeclareLaunchArgument(
            "yolo", default_value="false", description="Launch YOLO detection"
        ),
        DeclareLaunchArgument(
            "apriltag", default_value="false", description="Launch AprilTag detection"
        ),
    ]

    bringup_launch = FindPackageShare("colman_bringup")
    gripper_type = LaunchConfiguration("gripper_type")
    use_camera = LaunchConfiguration("use_camera")

    # call the template bringup for common nodes
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_launch, "launch", "bringup.launch.py"])
        ),
        launch_arguments={
            "start_external_rm": "false",
            "use_sim_time": LaunchConfiguration("sim"),
            "gripper_type": gripper_type,
            "use_camera": use_camera,
        }.items(),
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_launch, "launch", "sim.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    depth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_launch, "launch", "depth.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("depth")),
    )

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_launch, "launch", "yolo.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("yolo")),
    )

    apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_launch, "launch", "apriltag.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("apriltag")),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_launch, "launch", "moveit.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("moveit")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("sim"),
            "gripper_type": gripper_type,
            "use_camera": use_camera,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [bringup, sim, moveit, depth, yolo, apriltag]
    )
