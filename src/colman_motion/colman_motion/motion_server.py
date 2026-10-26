import os
import signal
import threading
import time
import traceback

import rclpy
from geometry_msgs.msg import Point
from moveit.planning import MoveItPy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from colman_interfaces.action import MovePose, MoveTo, Pick, Place
from colman_motion.arm_control import ArmControl
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager
from colman_motion.tag_lookup import TagLookup
from colman_motion.vacuum_control import VacuumControl

LOOK_POSE = "Look3"
DETECT_TIMEOUT_S = 10.0
APPROACH_OFFSET_M = 0.1
EE_OFFSET_M = 0.01
DROP_OFFSET_M = 0.05
EE_DOWN = (1.0, 0.0, 0.0, 0.0)
GRASP_DWELL_S = 0.5
RELEASE_DWELL_S = 0.5
CANCEL_WAIT_S = 0.5
SHUTDOWN_WAIT_S = 5.0
RESULT_FLUSH_S = 0.5
SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


class MotionServer(Node):
    def __init__(self, arm, tag, vacuum):
        super().__init__("motion_server")
        self.arm = arm
        self.tag = tag
        self.vacuum = vacuum

        group = ReentrantCallbackGroup()
        self.move_server = ActionServer(
            self,
            MoveTo,
            "move_to",
            execute_callback=self.move_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=group,
        )
        self.move_pose_server = ActionServer(
            self,
            MovePose,
            "move_pose",
            execute_callback=self.move_pose_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=group,
        )
        self.pick_server = ActionServer(
            self,
            Pick,
            "pick",
            execute_callback=self.pick_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=group,
        )
        self.place_server = ActionServer(
            self,
            Place,
            "place",
            execute_callback=self.place_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=group,
        )
        self.get_logger().info("/move_to, /move_pose, /pick and /place action servers started")

    def goal_cb(self, request):
        with self.arm.busy_lock:
            if self.arm.shutting_down:
                self.get_logger().warn("Rejecting goal, shutting down")
                return GoalResponse.REJECT
            if self.arm.stopped:
                self.get_logger().warn("Rejecting goal, stopped until /resume")
                return GoalResponse.REJECT
            if self.arm.busy:
                self.get_logger().warn("Rejecting goal, arm busy")
                return GoalResponse.REJECT
            self.arm.busy = True
            self.arm.stop_event.clear()
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel requested")
        self.arm.request_stop()
        threading.Thread(target=self.arm.halt, daemon=True).start()
        return CancelResponse.ACCEPT

    def stage(self, goal_handle, feedback, name):
        feedback.stage = name
        goal_handle.publish_feedback(feedback)

    def cancel_requested(self, goal_handle):
        deadline = time.monotonic() + CANCEL_WAIT_S
        while (
            self.arm.stop_event.is_set()
            and not self.arm.stopped
            and not goal_handle.is_cancel_requested
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        return goal_handle.is_cancel_requested

    def finish(self):
        with self.arm.busy_lock:
            self.arm.running = False
            self.arm.busy = False

    def fail(self, goal_handle, result, message):
        self.get_logger().error(message)
        result.message = message
        canceled = self.cancel_requested(goal_handle)
        self.finish()
        if canceled:
            goal_handle.canceled(result)
        else:
            goal_handle.abort(result)
        return result

    def detect(self, tag_frame):
        deadline = time.time() + DETECT_TIMEOUT_S
        while time.time() < deadline:
            if self.arm.stop_event.is_set():
                return None
            tag_pose = self.tag.get_tag_pose(tag_frame)
            if tag_pose is not None:
                return tag_pose
        return None

    def move_cb(self, goal_handle):
        result = MoveTo.Result()
        self.arm.running = True
        try:
            result.success = self.arm.go_to_configured_pose(
                goal_handle.request.pose_name, self.arm.fast
            )
            canceled = not result.success and self.cancel_requested(goal_handle)
            self.finish()
            if result.success:
                goal_handle.succeed(result)
            elif canceled:
                goal_handle.canceled(result)
            else:
                goal_handle.abort(result)
            return result
        finally:
            if goal_handle.is_active:
                self.finish()

    def move_pose_cb(self, goal_handle):
        result = MovePose.Result()
        self.arm.running = True
        try:
            pose = goal_handle.request.pose
            params = self.arm.slow if goal_handle.request.slow else self.arm.fast
            position = pose.position
            orientation = pose.orientation
            result.success = self.arm.go_to_pose(
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
                params,
            )
            if not result.success:
                result.message = "Move failed"
            canceled = not result.success and self.cancel_requested(goal_handle)
            self.finish()
            if result.success:
                goal_handle.succeed(result)
            elif canceled:
                goal_handle.canceled(result)
            else:
                goal_handle.abort(result)
            return result
        finally:
            if goal_handle.is_active:
                self.finish()

    def pick_cb(self, goal_handle):
        feedback = Pick.Feedback()
        result = Pick.Result()
        self.arm.running = True
        try:
            tag_frame = goal_handle.request.frame
            if self.vacuum.holding:
                return self.fail(goal_handle, result, "Already holding, call /release")

            self.stage(goal_handle, feedback, "looking")
            if not self.arm.go_to_configured_pose(LOOK_POSE, self.arm.fast):
                return self.fail(goal_handle, result, "Look move failed")

            self.stage(goal_handle, feedback, "detecting")
            tag_pose = self.detect(tag_frame)
            if tag_pose is None and self.arm.stop_event.is_set():
                return self.fail(goal_handle, result, "Stopped during detect")
            if tag_pose is None:
                return self.fail(goal_handle, result, f"{tag_frame} not visible")

            tag_translation = tag_pose.translation
            pick_translation = (
                tag_translation.x,
                tag_translation.y,
                tag_translation.z - EE_OFFSET_M,
            )
            approach_translation = (
                tag_translation.x,
                tag_translation.y,
                tag_translation.z + APPROACH_OFFSET_M,
            )

            self.stage(goal_handle, feedback, "approaching")
            if not self.arm.go_to_pose(*approach_translation, *EE_DOWN, self.arm.fast):
                return self.fail(goal_handle, result, "Approach failed")

            self.stage(goal_handle, feedback, "lowering")
            if not self.arm.go_to_pose(*pick_translation, *EE_DOWN, self.arm.slow):
                return self.fail(goal_handle, result, "Lower failed")

            self.stage(goal_handle, feedback, "grasping")
            if self.arm.stop_event.is_set():
                return self.fail(goal_handle, result, "Stopped before grasp")
            if not self.vacuum.grasp():
                return self.fail(goal_handle, result, "Vacuum grasp failed")
            time.sleep(GRASP_DWELL_S)

            self.stage(goal_handle, feedback, "retreating")
            if not self.arm.go_to_pose(*approach_translation, *EE_DOWN, self.arm.fast):
                return self.fail(goal_handle, result, "Retreat failed, still holding")

            result.success = True
            result.message = f"Picked {tag_frame}"
            self.finish()
            goal_handle.succeed(result)
            return result
        finally:
            if goal_handle.is_active:
                self.finish()

    def place_cb(self, goal_handle):
        feedback = Place.Feedback()
        result = Place.Result()
        self.arm.running = True
        try:
            location = goal_handle.request.location
            if not location and goal_handle.request.position == Point():
                return self.fail(goal_handle, result, "No location or position given")
            if not self.vacuum.holding:
                return self.fail(goal_handle, result, "Not holding anything")

            if location:
                self.stage(goal_handle, feedback, "looking")
                if not self.arm.go_to_configured_pose(LOOK_POSE, self.arm.fast):
                    return self.fail(goal_handle, result, "Look move failed")

                self.stage(goal_handle, feedback, "detecting")
                tag_pose = self.detect(location)
                if tag_pose is None and self.arm.stop_event.is_set():
                    return self.fail(goal_handle, result, "Stopped during detect")
                if tag_pose is None:
                    return self.fail(goal_handle, result, f"{location} not visible")
                drop_pose = tag_pose.translation
                drop_off = (
                    drop_pose.x,
                    drop_pose.y,
                    drop_pose.z + DROP_OFFSET_M,
                )
            else:
                position = goal_handle.request.position
                drop_off = (position.x, position.y, position.z)

            drop_off_approach = (
                drop_off[0],
                drop_off[1],
                drop_off[2] + APPROACH_OFFSET_M,
            )

            self.stage(goal_handle, feedback, "approaching")
            if not self.arm.go_to_pose(*drop_off_approach, *EE_DOWN, self.arm.fast):
                return self.fail(goal_handle, result, "Approach failed")

            self.stage(goal_handle, feedback, "lowering")
            if not self.arm.go_to_pose(*drop_off, *EE_DOWN, self.arm.slow):
                return self.fail(goal_handle, result, "Lower failed")

            self.stage(goal_handle, feedback, "releasing")
            if self.arm.stop_event.is_set():
                return self.fail(goal_handle, result, "Stopped before release")
            if not self.vacuum.release():
                return self.fail(goal_handle, result, "Vacuum release failed")
            time.sleep(RELEASE_DWELL_S)

            self.stage(goal_handle, feedback, "retreating")
            if not self.arm.go_to_pose(*drop_off_approach, *EE_DOWN, self.arm.slow):
                return self.fail(goal_handle, result, "Retreat failed")

            result.success = True
            if location:
                result.message = f"Placed at {location}"
            else:
                result.message = f"Placed at {drop_off}"
            self.finish()
            goal_handle.succeed(result)
            return result
        finally:
            if goal_handle.is_active:
                self.finish()


def on_signal(signum, frame):
    for sig in SIGNALS:
        signal.signal(sig, signal.SIG_IGN)
    raise KeyboardInterrupt


def main():
    for sig in SIGNALS:
        signal.signal(sig, on_signal)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    try:
        wait_for_robot()
        ur = MoveItPy(node_name="moveit_py")
        for sig in SIGNALS:
            signal.signal(sig, on_signal)

        arm = ArmControl(ur)
        scene = SceneManager(ur)
        tag = TagLookup()
        vacuum = VacuumControl()
        motion = MotionServer(arm, tag, vacuum)

        executor = MultiThreadedExecutor()
        executor.add_node(arm)
        executor.add_node(scene)
        executor.add_node(tag)
        executor.add_node(vacuum)
        executor.add_node(motion)

        scene.add_box(
            "table",
            "base_link",
            [2.0, 2.0, 0.01],
            (0.0, 0.0, -0.01),
        )
    except KeyboardInterrupt:
        os._exit(1)
    except Exception:
        traceback.print_exc()
        os._exit(1)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)

    released = False
    try:
        spin_thread.start()
        vacuum.release()
        while rclpy.ok() and spin_thread.is_alive():
            time.sleep(0.5)
        if not spin_thread.is_alive():
            motion.get_logger().error("Executor stopped, shutting down")
    except KeyboardInterrupt:
        pass
    finally:
        for sig in SIGNALS:
            signal.signal(sig, signal.SIG_IGN)
        with arm.busy_lock:
            arm.shutting_down = True
            arm.stopped = True
        arm.request_stop()
        threading.Thread(target=arm.halt, daemon=True).start()
        deadline = time.monotonic() + SHUTDOWN_WAIT_S
        while arm.busy and time.monotonic() < deadline:
            time.sleep(0.05)
        time.sleep(RESULT_FLUSH_S)
        try:
            released = vacuum.release()
            scene.clear_scene()
            executor.shutdown(timeout_sec=1.0)
            rclpy.shutdown()
        except Exception:
            traceback.print_exc()
        os._exit(0 if released else 1)


if __name__ == "__main__":
    main()
