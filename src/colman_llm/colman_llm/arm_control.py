"""Runs a validated move_arm tool call as a /move_to action goal."""

import threading

from rclpy.action import ActionClient

from colman_interfaces.action import MoveTo
from colman_llm.tools import POSITIONS

SERVER_WAIT_S = 2.0
MOVE_TIMEOUT_S = 60.0


class ArmControl:
    def __init__(self, node):
        self._client = ActionClient(node, MoveTo, "move_to")

    def run_tool(self, name, args):
        """Return (ok, message) for a tool call. Always re-checks the arguments."""
        if name != "move_arm":
            return False, f"Unknown tool {name}."
        pose = POSITIONS.get(args.get("position"))
        if pose is None:
            return False, "Unknown position."
        return self._move_to(pose)

    def _move_to(self, pose):
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            return False, "The arm controller is not running."
        goal = MoveTo.Goal()
        goal.pose_name = pose
        done = threading.Event()
        box = {}

        def on_result(future):
            box["result"] = future.result()
            done.set()

        def on_goal(future):
            handle = future.result()
            box["handle"] = handle
            if not handle.accepted:
                done.set()
                return
            handle.get_result_async().add_done_callback(on_result)

        self._client.send_goal_async(goal).add_done_callback(on_goal)
        if not done.wait(MOVE_TIMEOUT_S):
            handle = box.get("handle")
            if handle is not None:
                handle.cancel_goal_async()
            return False, "The move timed out and was cancelled."
        if "result" not in box:
            return False, "The arm rejected the move. It may be busy or stopped."
        if not box["result"].result.success:
            return False, "The move failed."
        return True, "Moved."
