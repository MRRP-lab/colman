import threading

from geometry_msgs.msg import PoseStamped
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import PlanRequestParameters
from rclpy.node import Node
from std_srvs.srv import Trigger

FAST_SCALING = 0.7
SLOW_SCALING = 0.3


class ArmControl(Node):
    def __init__(self, moveit):
        super().__init__("arm_control")
        self.moveit = moveit
        self.arm = moveit.get_planning_component("ur_arm")
        self.trajectory_execution_manager = moveit.get_trajectory_execution_manager()

        self.fast = PlanRequestParameters(moveit, "ompl")
        self.fast.max_velocity_scaling_factor = FAST_SCALING
        self.fast.max_acceleration_scaling_factor = FAST_SCALING

        self.slow = PlanRequestParameters(moveit, "ompl")
        self.slow.max_velocity_scaling_factor = SLOW_SCALING
        self.slow.max_acceleration_scaling_factor = SLOW_SCALING
        self.stop_event = threading.Event()
        self.stopped = False
        self.shutting_down = False
        self.busy = False
        self.running = False
        self.busy_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.create_service(Trigger, "/stop", self.stop_cb)
        self.create_service(Trigger, "/resume", self.resume_cb)
        self.get_logger().info("/stop and /resume services started")

    def stop_cb(self, request, response):
        self.get_logger().warn("Stop activated")
        with self.busy_lock:
            self.stopped = True
        self.stop()
        response.success = True
        response.message = "Successfully Stopped"
        return response

    def resume_cb(self, request, response):
        self.get_logger().warn("Resume activated")
        with self.busy_lock:
            if self.stopped and not self.running:
                self.busy = False
            self.stopped = False
        response.success = True
        response.message = "Successfully Resumed"
        return response

    def stop(self):
        self.request_stop()
        self.halt()

    def request_stop(self):
        with self.busy_lock:
            self.stop_event.set()

    def halt(self):
        with self.execute_lock:
            if self.stop_event.is_set():
                self.trajectory_execution_manager.stop_execution(auto_clear=True)

    def execute(self, robot_trajectory):
        tem = self.trajectory_execution_manager
        with self.execute_lock:
            if self.stop_event.is_set():
                return False
            if not tem.ensure_active_controllers_for_group("ur_arm"):
                self.get_logger().error("No active controller for ur_arm")
                return False
            self.get_logger().info("Executing plan")
            if not tem.push(robot_trajectory.get_robot_trajectory_msg(), []):
                self.get_logger().error("Trajectory push failed")
                return False
            done = threading.Event()
            tem.execute(lambda status: done.set())
        done.wait()
        status = tem.wait_for_execution()
        if not status:
            self.get_logger().error(f"Execution {status.status}")
        return bool(status) and not self.stop_event.is_set()

    def plan_and_execute(
        self,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        retries=3,
    ):
        for _ in range(retries):
            if self.stop_event.is_set():
                return False
            if multi_plan_parameters is not None:
                plan_result = self.arm.plan(multi_plan_parameters=multi_plan_parameters)
            elif single_plan_parameters is not None:
                plan_result = self.arm.plan(
                    single_plan_parameters=single_plan_parameters
                )
            else:
                plan_result = self.arm.plan()

            if plan_result:
                return self.execute(plan_result.trajectory)

        self.get_logger().error("Planning failed")
        return False

    def go_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0, params=None):
        self.arm.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        if not self.arm.set_goal_state(
            pose_stamped_msg=pose_goal, pose_link="vacuum_tip_link"
        ):
            return False
        return self.plan_and_execute(single_plan_parameters=params)

    def go_to_configured_pose(self, pose_name, params=None):
        self.arm.set_start_state_to_current_state()
        if not self.arm.set_goal_state(configuration_name=pose_name):
            return False
        return self.plan_and_execute(single_plan_parameters=params)

    def go_to_joint_pose(self, joint_values, params=None):
        self.arm.set_start_state_to_current_state()
        robot_model = self.moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=robot_model.get_joint_model_group("ur_arm"),
        )
        if not self.arm.set_goal_state(motion_plan_constraints=[joint_constraint]):
            return False
        return self.plan_and_execute(single_plan_parameters=params)
