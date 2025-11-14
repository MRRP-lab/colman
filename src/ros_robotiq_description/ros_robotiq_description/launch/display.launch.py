from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ros_robotiq_description')

    # import our rviz config
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display_config.rviz')

    # import our xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'ur3e_robotiq.urdf.xacro')

    # convert xacro to urdf
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        rsp_node,
        joint_state_gui,
        rviz_node,
    ])
