import os
import signal
import threading
import time
import traceback

import rclpy
from colman_description.workcell import CUBE_SIZE_M, TABLE_TOP_Z
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
from colman_motion.scene_manager import SceneManager
from colman_motion.tag_lookup import TagLookup
from colman_motion.vacuum_control import VacuumControl
from colman_motion.world_scene import WorldScene

REFOCUS_SERVICE = "/oakd_pro/refocus"
FIND_TIMEOUT_S = 4.0
APPROACH_OFFSET_M = 0.1
EE_OFFSET_M = 0.01
# The cube is let go this far above the top of the one it goes on: its own height, so
# that its bottom meets the top of the cube below.
DROP_OFFSET_M = CUBE_SIZE_M
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


class MotionServer(Node):
    def __init__(self, arm, tag, vacuum, world=None):
        super().__init__("motion_server")
        self.arm = arm
        self.tag = tag
        self.vacuum = vacuum
        # Keeps the cubes and stacks in the planning scene. None: the scene has only the
        # table and the cube in the gripper.
        self.world = world
        # Asks the wrist camera for a fresh autofocus and autoexposure pass.
        self.refocus = self.create_client(Trigger, REFOCUS_SERVICE)

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

    def keep_clear(self, tags):
        """Leave the columns holding these tags out of the planning scene for a move."""
        if self.world is None:
            return
        if not self.world.current_world():
            self.get_logger().warn("No current world state: planning without the cubes")
        self.world.exclude(tags)

    def stop_keeping_clear(self):
        if self.world is not None:
            self.world.clear_exclusions()

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

    def request_refocus(self):
        # The arm has just moved, so let the camera refocus and re-expose while we wait.
        if self.refocus.service_is_ready():
            self.refocus.call_async(Trigger.Request())

    def find_tag(self, goal_handle, feedback, tag_frame):
        """Find tag_frame without a scan pose. Return (tag_pose, error_message).

        The wrist camera is tried first, then the overhead camera. If neither sees it
        the arm may be in the way, so it moves clear and the overhead is tried again.
        """
        self.stage(goal_handle, feedback, "detecting")
        self.request_refocus()
        deadline = time.monotonic() + FIND_TIMEOUT_S
        while not self.arm.stop_event.is_set():
            tag_pose = self.tag.get_tag_pose(tag_frame)
            if tag_pose is not None:
                return tag_pose, None
            if time.monotonic() >= deadline:
                return None, f"{tag_frame} not visible"
        return None, "Stopped during detect"

    def move_cb(self, goal_handle):
        result = MoveTo.Result()
        self.arm.running = True
        try:
            result.success = self.arm.go_to_configured_pose(
                goal_handle.request.pose_name, self.arm.fast
            )
            canceled = not result.success and self.cancel_requested(goal_handle)
            if not result.success and not canceled:
                # The result has no message field, so say why in the log.
                self.get_logger().error(
                    f"Move to {goal_handle.request.pose_name} failed"
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
            # Nothing is held, so no cube is modelled while picking, and the column
            # being picked from is not an obstacle the arm may not touch.
            self.arm.detach_cube()
            self.keep_clear([tag_frame])

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
            approach_state, _, tool = pair

            self.stage(goal_handle, feedback, "approaching")
            if not self.arm.move_to_state(approach_state, self.arm.fast):
                return self.fail(goal_handle, result, "Approach failed")

            self.stage(goal_handle, feedback, "lowering")
            if not self.arm.go_straight(*pick_translation, tool, self.arm.slow):
                return self.fail(goal_handle, result, "Lower failed")

            self.stage(goal_handle, feedback, "grasping")
            if self.arm.stop_event.is_set():
                return self.fail(goal_handle, result, "Stopped before grasp")
            if not self.vacuum.grasp():
                return self.fail(goal_handle, result, "Vacuum grasp failed")
            time.sleep(GRASP_DWELL_S)

            self.stage(goal_handle, feedback, "retreating")
            if not self.arm.go_straight(*approach_translation, tool, self.arm.fast):
                return self.fail(goal_handle, result, "Retreat failed, still holding")

            result.success = True
            result.message = f"Picked {tag_frame}"
            self.finish()
            goal_handle.succeed(result)
            return result
        finally:
            self.stop_keeping_clear()
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
            # The cube is part of the arm from here until it is released, including
            # a move out of the overhead camera's view.
            self.arm.attach_cube()
            if location:
                self.keep_clear([location])

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
            approach_state, _, tool = pair

            self.stage(goal_handle, feedback, "approaching")
            if not self.arm.move_to_state(approach_state, self.arm.fast):
                return self.fail(goal_handle, result, "Approach failed")

            self.stage(goal_handle, feedback, "lowering")
            if not self.arm.go_straight(*drop_off, tool, self.arm.slow):
                return self.fail(goal_handle, result, "Lower failed")

            self.stage(goal_handle, feedback, "releasing")
            if self.arm.stop_event.is_set():
                return self.fail(goal_handle, result, "Stopped before release")
            if not self.vacuum.release():
                return self.fail(goal_handle, result, "Vacuum release failed")
            self.arm.detach_cube()
            time.sleep(RELEASE_DWELL_S)

            self.stage(goal_handle, feedback, "retreating")
            if not self.arm.go_straight(*drop_off_approach, tool, self.arm.slow):
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
            self.stop_keeping_clear()
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
        world = WorldScene(scene)
        motion = MotionServer(arm, tag, vacuum, world)

        executor = MultiThreadedExecutor()
        executor.add_node(arm)
        executor.add_node(scene)
        executor.add_node(tag)
        executor.add_node(vacuum)
        executor.add_node(world)
        executor.add_node(motion)

        scene.add_box(
            "table",
            "base_link",
            [2.0, 2.0, 0.01],
            (0.0, 0.0, TABLE_TOP_Z - 0.005),
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
