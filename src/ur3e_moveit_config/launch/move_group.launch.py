from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 1. Build the config object
    moveit_config = MoveItConfigsBuilder(
        "ur3_robotiq", package_name="ur3e_moveit_config"
    ).to_moveit_configs()

    # 2. Convert to dictionary so we can modify it
    moveit_config_dict = moveit_config.to_dict()

    # 3. Inject the MTC Capability
    # Checks if 'capabilities' exists to avoid overwriting other plugins
    if "capabilities" in moveit_config_dict:
        moveit_config_dict[
            "capabilities"
        ] += " move_group/ExecuteTaskSolutionCapability"
    else:
        moveit_config_dict["capabilities"] = "move_group/ExecuteTaskSolutionCapability"

    moveit_config_dict["use_sim_time"] = True
    moveit_config_dict["publish_robot_description_semantic"] = True
    # 4. Define the MoveGroup Node Manually
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict, moveit_config.trajectory_execution],
    )

    # 5. Return the standard LaunchDescription
    return LaunchDescription([run_move_group_node])
