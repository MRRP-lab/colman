import rclpy
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from moveit.core.kinematic_constraints import construct_joint_constraint


HOME = "Up"

UP = {
        "elbow_joint": 0,
        "shoulder_lift_joint": -1.57,
        "shoulder_pan_joint": 0,
        "wrist_1_joint": 0,
        "wrist_2_joint": 1.59,
        "wrist_3_joint": 0,
    }

LOOK_ONE = {
        "elbow_joint": -0.6769,
        "shoulder_lift_joint": -1.4927,
        "shoulder_pan_joint": 0,
        "wrist_1_joint": -2.3952,
        "wrist_2_joint": 1.6315,
        "wrist_3_joint": 0,
    }

LOOK_FOUR = {
        "elbow_joint": -0.6769,
        "shoulder_lift_joint": -1.4927,
        "shoulder_pan_joint": -1.5708,
        "wrist_1_joint": -2.3952,
        "wrist_2_joint": 1.6315,
        "wrist_3_joint": 0,
    }

COMPACT = {
        "elbow_joint": -2.2043,
        "shoulder_lift_joint": -0.5901,
        "shoulder_pan_joint": 0,
        "wrist_1_joint": -0.3818,
        "wrist_2_joint": 0,
        "wrist_3_joint": 0,
    }

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    ):
    """A helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
        return True
    logger.error("Planning failed")
    return False
            
def go_to_pose(ur, arm, logger, x, y, z) -> bool: 
    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = x
    pose_goal.pose.position.y = y
    pose_goal.pose.position.z = z
    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")

    # plan to goal
    return plan_and_execute(ur, arm, logger)


def go_to_configured_pose(ur, arm, logger, pose_name) -> bool:
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=pose_name)
    return plan_and_execute(ur, arm, logger)

def go_to_joint_pose(ur, arm, logger, joint_values) -> bool:
    # set plan start state to current state
    arm.set_start_state_to_current_state()

    robot_model = ur.get_robot_model()
    robot_state = RobotState(robot_model)

    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=ur.get_robot_model().get_joint_model_group("ur_arm"),
    )
    arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    return plan_and_execute(ur, arm, logger)

def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")
    ur = MoveItPy(node_name="moveit_py")
    arm = ur.get_planning_component("ur_arm")
    logger.info("MoveItPy instance created")

    # go home
    if not go_to_configured_pose(ur, arm, logger, "up"):
        return

    if not go_to_configured_pose(ur, arm, logger, "test_configuration"):
        return

    #if not go_to_joint_pose(ur, arm, logger, UP):
       # return
    #if not go_to_joint_pose(ur, arm, logger, LOOK_ONE):
       # return


if __name__ == "__main__":
    main()
