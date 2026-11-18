import functools
import os
import signal
import sys
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
from std_srvs.srv import Trigger

from colman_interfaces.action import MovePose, MoveTo, Pick, Place
from colman_motion.arm_control import ArmControl
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scan import FOUND, NOT_VISIBLE, STOPPED, locate_tag
from colman_motion.scene_manager import SceneManager
from colman_motion.tag_lookup import TagLookup
from colman_motion.vacuum_control import VacuumControl

REFOCUS_SERVICE = "/oakd_pro/refocus"
APPROACH_OFFSET_M = 0.1
EE_OFFSET_M = 0.01
DROP_OFFSET_M = 0.05
# The approach above the drop point is lower than the one above a pick: with the cube
# in hand a 0.10 m approach over the third cube of a stack is out of the arm's reach.
PLACE_APPROACH_OFFSET_M = 0.05
EE_DOWN = (1.0, 0.0, 0.0, 0.0)
GRASP_DWELL_S = 0.5
RELEASE_DWELL_S = 0.5
CANCEL_WAIT_S = 0.5
SHUTDOWN_WAIT_S = 5.0
RESULT_FLUSH_S = 0.5
SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def guarded(result_type):
    """Turn an exception in a goal callback into an abort that says what happened.

    Left alone, rclpy logs the traceback and aborts the goal with an empty result,
    so the client sees a failure it cannot explain.
    """

    def decorate(callback):
        @functools.wraps(callback)
        def wrapper(self, goal_handle):
            try:
                return callback(self, goal_handle)
            except Exception as error:
                self.get_logger().error(traceback.format_exc())
                result = result_type()
                message = f"Internal error: {type(error).__name__}: {error}"
                if hasattr(result, "message"):
                    result.message = message
                self.get_logger().error(
                    f"Abort: {message} [{self.describe(goal_handle)}]"
                )
                self.finish()
                if goal_handle.is_active:
                    goal_handle.abort(result)
                return result

        return wrapper

    return decorate


class MotionServer(Node):
    def __init__(self, arm, tag, vacuum):
        super().__init__("motion_server")
        self.arm = arm
        self.tag = tag
        self.vacuum = vacuum
        # Asks the wrist camera for a fresh autofocus and autoexposure pass.
        self.refocus = self.create_client(Trigger, REFOCUS_SERVICE)
        self.current_stage = "idle"
        # Letting go of the cube, however that happens, takes it off the arm's model.
        self.vacuum.on_release = self.arm.detach_cube

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
        self.get_logger().info(
            "/move_to, /move_pose, /pick and /place action servers started"
        )

    def goal_cb(self, request):
        with self.arm.busy_lock:
            if self.arm.shutting_down:
                self.get_logger().warn("Rejecting goal, shutting down")
                return GoalResponse.REJECT
            if self.arm.wedged:
                self.get_logger().warn("Rejecting goal, restarting after a stuck move")
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

    def begin(self, goal_handle):
        """Mark the start of a goal: reset the stage and the last failure."""
        self.arm.running = True
        self.arm.last_failure = ""
        self.current_stage = "starting"

    def describe(self, goal_handle):
        goal_id = bytes(goal_handle.goal_id.uuid).hex()[:8]
        failure = self.arm.last_failure or "none"
        return f"goal={goal_id} stage={self.current_stage} moveit={failure}"

    def stage(self, goal_handle, feedback, name):
        self.current_stage = name
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
        if self.arm.wedged:
            message = f"{message}: {self.arm.wedged}, the motion server is restarting"
        self.get_logger().error(f"Abort: {message} [{self.describe(goal_handle)}]")
        if self.arm.last_failure:
            message = f"{message}: {self.arm.last_failure}"
        result.message = message
        canceled = self.cancel_requested(goal_handle)
        self.finish()
        if canceled:
            goal_handle.canceled(result)
        else:
            goal_handle.abort(result)
        return result

    def request_refocus(self):
        # The arm has just moved, so let the camera refocus and re-expose while we wait.
        if self.refocus.service_is_ready():
            self.refocus.call_async(Trigger.Request())

    def find_tag(self, goal_handle, feedback, tag_frame):
        """Find tag_frame without a scan pose. Return (tag_pose, error_message).

        The wrist camera is tried first, then the overhead camera. If neither sees it
        the arm may be in the way, so it moves clear and the overhead is tried again.
        """

        def on_detect():
            self.stage(goal_handle, feedback, "detecting")
            self.request_refocus()

        scan = locate_tag(
            self.arm,
            self.tag,
            tag_frame,
            self.arm.fast,
            stopped=self.arm.stop_event.is_set,
            on_look=lambda: self.stage(goal_handle, feedback, "looking"),
            on_detect=on_detect,
        )
        if scan.status == FOUND:
            return scan.tag_pose, None
        if scan.status == STOPPED:
            return None, "Stopped during detect"
        if scan.status == NOT_VISIBLE:
            return None, f"{tag_frame} not visible"
        return None, scan.message

    @guarded(MoveTo.Result)
    def move_cb(self, goal_handle):
        result = MoveTo.Result()
        self.begin(goal_handle)
        try:
            result.success = self.arm.go_to_configured_pose(
                goal_handle.request.pose_name, self.arm.fast
            )
            canceled = not result.success and self.cancel_requested(goal_handle)
            if not result.success and not canceled:
                # The result has no message field, so say why in the log.
                self.get_logger().error(
                    f"Abort: move to {goal_handle.request.pose_name} failed "
                    f"[{self.describe(goal_handle)}]"
                )
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

    @guarded(MovePose.Result)
    def move_pose_cb(self, goal_handle):
        result = MovePose.Result()
        self.begin(goal_handle)
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
                result.message = f"Move failed: {self.arm.last_failure or 'unknown'}"
                self.get_logger().error(
                    f"Abort: {result.message} [{self.describe(goal_handle)}]"
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

    @guarded(Pick.Result)
    def pick_cb(self, goal_handle):
        feedback = Pick.Feedback()
        result = Pick.Result()
        self.begin(goal_handle)
        try:
            tag_frame = goal_handle.request.frame
            if self.vacuum.holding:
                return self.fail(goal_handle, result, "Already holding, call /release")
            # Nothing is held, so no cube is modelled while picking.
            self.arm.detach_cube()

            tag_pose, error = self.find_tag(goal_handle, feedback, tag_frame)
            if error:
                return self.fail(goal_handle, result, error)

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

            # Both poses are solved on one IK branch, so the drop is a short straight
            # move and not a large sweep of the arm.
            pair = self.arm.vertical_pair(
                pick_translation[0],
                pick_translation[1],
                pick_translation[2],
                approach_translation[2],
                EE_DOWN,
            )
            if pair is None:
                return self.fail(
                    goal_handle, result, f"No safe arm pose to reach {tag_frame}"
                )
            approach_state, _ = pair

            self.stage(goal_handle, feedback, "approaching")
            if not self.arm.move_to_state(approach_state, self.arm.fast):
                return self.fail(goal_handle, result, "Approach failed")

            self.stage(goal_handle, feedback, "lowering")
            if not self.arm.go_straight(*pick_translation, EE_DOWN, self.arm.slow):
                return self.fail(goal_handle, result, "Lower failed")

            self.stage(goal_handle, feedback, "grasping")
            if self.arm.stop_event.is_set():
                return self.fail(goal_handle, result, "Stopped before grasp")
            if not self.vacuum.grasp():
                return self.fail(goal_handle, result, "Vacuum grasp failed")
            time.sleep(GRASP_DWELL_S)

            self.stage(goal_handle, feedback, "retreating")
            if not self.arm.go_straight(*approach_translation, EE_DOWN, self.arm.fast):
                return self.fail(goal_handle, result, "Retreat failed, still holding")

            result.success = True
            result.message = f"Picked {tag_frame}"
            self.finish()
            goal_handle.succeed(result)
            return result
        finally:
            if goal_handle.is_active:
                self.finish()

    @guarded(Place.Result)
    def place_cb(self, goal_handle):
        feedback = Place.Feedback()
        result = Place.Result()
        self.begin(goal_handle)
        try:
            location = goal_handle.request.location
            if not location and goal_handle.request.position == Point():
                return self.fail(goal_handle, result, "No location or position given")
            if not self.vacuum.holding:
                return self.fail(goal_handle, result, "Not holding anything")
            # The cube is part of the arm from here until it is released, including
            # a move out of the overhead camera's view.
            self.arm.attach_cube()

            if location:
                tag_pose, error = self.find_tag(goal_handle, feedback, location)
                if error:
                    return self.fail(goal_handle, result, error)
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
                drop_off[2] + PLACE_APPROACH_OFFSET_M,
            )

            pair = self.arm.vertical_pair(
                drop_off[0],
                drop_off[1],
                drop_off[2],
                drop_off_approach[2],
                EE_DOWN,
            )
            if pair is None:
                return self.fail(goal_handle, result, "No safe arm pose to place there")
            approach_state, _ = pair

            self.stage(goal_handle, feedback, "approaching")
            if not self.arm.move_to_state(approach_state, self.arm.fast):
                return self.fail(goal_handle, result, "Approach failed")

            self.stage(goal_handle, feedback, "lowering")
            if not self.arm.go_straight(*drop_off, EE_DOWN, self.arm.slow):
                return self.fail(goal_handle, result, "Lower failed")

            self.stage(goal_handle, feedback, "releasing")
            if self.arm.stop_event.is_set():
                return self.fail(goal_handle, result, "Stopped before release")
            if not self.vacuum.release():
                return self.fail(goal_handle, result, "Vacuum release failed")
            self.arm.detach_cube()
            time.sleep(RELEASE_DWELL_S)

            self.stage(goal_handle, feedback, "retreating")
            if not self.arm.go_straight(*drop_off_approach, EE_DOWN, self.arm.slow):
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


def restart_process(arm, vacuum, logger):
    """Replace this process with a fresh motion server.

    MoveIt's execution manager cannot recover once a controller stopped answering in
    the middle of a trajectory, so the only way to take goals again is a new process.
    The goal in flight has been answered by now; give its result a moment to be sent.
    """
    logger.error(f"Restarting the motion server: {arm.wedged}")
    with arm.busy_lock:
        arm.shutting_down = True
    deadline = time.monotonic() + SHUTDOWN_WAIT_S
    while arm.busy and time.monotonic() < deadline:
        time.sleep(0.05)
    time.sleep(RESULT_FLUSH_S)
    vacuum.release()
    sys.stdout.flush()
    sys.stderr.flush()
    # Sockets and shared memory the middleware opened are not close-on-exec.
    for fd in os.listdir("/proc/self/fd"):
        if int(fd) > 2:
            try:
                os.close(int(fd))
            except OSError:
                pass
    os.execv(sys.executable, [sys.executable, *sys.orig_argv[1:]])


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
            if arm.wedged:
                restart_process(arm, vacuum, motion.get_logger())
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
