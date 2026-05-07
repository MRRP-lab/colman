import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import CollisionObject
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, ExtrapolationException, LookupException, TransformListener
from ur_msgs.srv import SetIO

HOME = "Up"

APPROACH_OFFSET = 0.05

EE_DOWN = (1.0, 0.0, 0.0, 0.0)

DROP_OFF = (0.2, -0.2, 0.0381)

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

ON = True
OFF = False


class StopControl(Node):
    def __init__(self, moveit):
        super().__init__("stop_service")
        self.trajectory_execution_manager = moveit.get_trajectory_execution_manager()
        self.stop_event = threading.Event()
        self.create_service(Trigger, "/stop", self.stop_cb)
        self.get_logger().info("/stop service started")

    def stop_cb(self, request, response):
        self.get_logger().warn("Stop activated")
        self.stop_event.set()
        self.trajectory_execution_manager.stop_execution(auto_clear=True)
        response.success = True
        response.message = "Successfully Stopped"
        return response


class SceneManager(Node):
    def __init__(self, moveit):
        super().__init__("scene_manager")
        self.planning_scene_monitor = moveit.get_planning_scene_monitor()

    def add_box(self, obj_id, frame_id, dims, pos, orientation_w=1.0):
        with self.planning_scene_monitor.read_write() as scene:
            obj = CollisionObject()
            obj.header.frame_id = frame_id
            obj.id = obj_id

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dims

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos
            pose.orientation.w = orientation_w

            obj.primitives.append(box)
            obj.primitive_poses.append(pose)
            obj.operation = CollisionObject.ADD

            scene.apply_collision_object(obj)
            scene.current_state.update()

        self.get_logger().info(f"Added: {obj_id}")

    def remove_object(self, obj_id):
        with self.planning_scene_monitor.read_write() as scene:
            obj = CollisionObject()
            obj.id = obj_id
            obj.operation = CollisionObject.REMOVE
            scene.apply_collision_object(obj)
            scene.current_state.update()
        self.get_logger().info(f"Removed: {obj_id}")

    def clear_scene(self):
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()
        self.get_logger().info("Cleared all objects")


class VacuumControl(Node):
    def __init__(self):
        super().__init__("vacuum_control")
        self.vacuum = self.create_client(
            SetIO,
            "/io_and_status_controller/set_io",
            callback_group=ReentrantCallbackGroup(),
        )

    def set_digital_out(self, pin, state):
        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin = pin
        req.state = float(state)
        future = self.vacuum.call_async(req)
        while not future.done():
            time.sleep(0.001)
        return future.result().success


class TagLookup(Node):
    def __init__(self):
        super().__init__("tag_lookup")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def lookup(self, tag_frame, base_frame):
        try:
            return self.buffer.lookup_transform(
                base_frame,
                tag_frame,
                Time(),
                Duration(seconds=2.0),
            )
        except (LookupException, ExtrapolationException):
            return None

    def get_tag_pose(self, tag_frame="tag_1", base_frame="base_link"):
        first = self.lookup(tag_frame, base_frame)

        error_m = 0.003  # 3mm
        wait_s = 1.0

        time.sleep(wait_s)

        second = self.lookup(tag_frame, base_frame)

        if first is None or second is None:
            return None

        if first.header.stamp == second.header.stamp:
            return None

        first_pos = first.transform.translation
        second_pos = second.transform.translation

        dx = abs(second_pos.x - first_pos.x)
        dy = abs(second_pos.y - first_pos.y)
        dz = abs(second_pos.z - first_pos.z)

        if max(dx, dy, dz) >= error_m:
            return None

        return second.transform


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    retries=3,
    stop_event=None,
):

    for attempt in range(retries):
        if stop_event and stop_event.is_set():
            return False
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

        if plan_result:
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            if robot.execute(robot_trajectory, controllers=[]):
                return True

    logger.error("Planning failed")
    return False


def go_to_pose(
    ur,
    arm,
    logger,
    x,
    y,
    z,
    qx=0.0,
    qy=0.0,
    qz=0.0,
    qw=1.0,
    params=None,
    stop_event=None,
):
    arm.set_start_state_to_current_state()
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.x = qx
    pose_goal.pose.orientation.y = qy
    pose_goal.pose.orientation.z = qz
    pose_goal.pose.orientation.w = qw
    pose_goal.pose.position.x = x
    pose_goal.pose.position.y = y
    pose_goal.pose.position.z = z
    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="vacuum_tip_link")
    return plan_and_execute(
        ur, arm, logger, single_plan_parameters=params, stop_event=stop_event
    )


def go_to_configured_pose(ur, arm, logger, pose_name):
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=pose_name)
    return plan_and_execute(ur, arm, logger)


def go_to_joint_pose(ur, arm, logger, joint_values, params=None, stop_event=None):
    arm.set_start_state_to_current_state()
    robot_model = ur.get_robot_model()
    robot_state = RobotState(robot_model)
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=robot_model.get_joint_model_group("ur_arm"),
    )
    arm.set_goal_state(motion_plan_constraints=[joint_constraint])
    return plan_and_execute(
        ur, arm, logger, single_plan_parameters=params, stop_event=stop_event
    )


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")
    ur = MoveItPy(node_name="moveit_py")
    arm = ur.get_planning_component("ur_arm")
    logger.info("MoveItPy instance created")

    stop = StopControl(ur)
    scene = SceneManager(ur)
    vacuum = VacuumControl()
    tag = TagLookup()

    executor = MultiThreadedExecutor()
    executor.add_node(stop)
    executor.add_node(scene)
    executor.add_node(vacuum)
    executor.add_node(tag)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    vacuum.vacuum.wait_for_service()

    try:
        scene.add_box(
            "table",
            "base_link",
            [2.0, 2.0, 0.01],
            (0.0, 0.0, -0.01),
        )

        fast = PlanRequestParameters(ur, "ompl")
        fast.max_velocity_scaling_factor = 0.3
        fast.max_acceleration_scaling_factor = 0.3

        slow = PlanRequestParameters(ur, "ompl")
        slow.max_velocity_scaling_factor = 0.1
        slow.max_acceleration_scaling_factor = 0.1

        # tool0 up Translation: [0.086, 0.129, 0.701]

        stop_event = stop.stop_event

        vacuum.set_digital_out(0, OFF)

        while not stop_event.is_set():
            if not go_to_joint_pose(
                ur, arm, logger, LOOK_ONE, slow, stop_event=stop_event
            ):
                logger.warn("Stopping")
                return
            tag_pose = tag.get_tag_pose()

            if tag_pose is None:
                continue

            tag_translation = tag_pose.translation
            pick_translation = (tag_translation.x, tag_translation.y, tag_translation.z)
            approach_translation = (
                tag_translation.x,
                tag_translation.y,
                tag_translation.z + APPROACH_OFFSET,
            )

            if not go_to_pose(
                ur,
                arm,
                logger,
                *approach_translation,
                *EE_DOWN,
                slow,
                stop_event=stop_event,
            ):
                logger.warn("Stopping")
                return

            if not go_to_pose(
                ur,
                arm,
                logger,
                *pick_translation,
                *EE_DOWN,
                slow,
                stop_event=stop_event,
            ):
                logger.warn("Stopping")
                return

            vacuum.set_digital_out(0, ON)

            if not go_to_pose(
                ur,
                arm,
                logger,
                *approach_translation,
                *EE_DOWN,
                slow,
                stop_event=stop_event,
            ):
                logger.warn("Stopping")
                return

            drop_off_approach = (
                DROP_OFF[0],
                DROP_OFF[1],
                DROP_OFF[2] + APPROACH_OFFSET,
            )

            if not go_to_pose(
                ur,
                arm,
                logger,
                *drop_off_approach,
                *EE_DOWN,
                slow,
                stop_event=stop_event,
            ):
                logger.warn("Stopping")
                return

            if not go_to_pose(
                ur, arm, logger, *DROP_OFF, *EE_DOWN, slow, stop_event=stop_event
            ):
                logger.warn("Stopping")
                return

            vacuum.set_digital_out(0, OFF)

            if not go_to_pose(
                ur,
                arm,
                logger,
                *drop_off_approach,
                *EE_DOWN,
                slow,
                stop_event=stop_event,
            ):
                logger.warn("Stopping")
                return

    finally:
        vacuum.set_digital_out(0, OFF)
        scene.clear_scene()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
