"""Blocking call of one ROS action from the worker thread.

The worker waits on a threading.Event while the node's executor thread delivers the
callbacks, so the node is never spun from the worker. The ActionClient is passed in,
which lets tests use a fake one without ROS.
"""

import threading

SERVER_WAIT_S = 2.0


class ActionCall:
    def __init__(self, client, name, logger):
        self._client = client
        self._name = name
        self._log = logger

    def call(self, goal, timeout_s):
        """Send goal and wait. Return (ok, message) in plain language."""
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            return False, "The arm controller is not running."

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

        def on_feedback(msg):
            stage = getattr(msg.feedback, "stage", "")
            self._log.info(f"{self._name}: {stage}")

        self._client.send_goal_async(
            goal, feedback_callback=on_feedback
        ).add_done_callback(on_goal)
        if not done.wait(timeout_s):
            handle = box.get("handle")
            if handle is not None:
                handle.cancel_goal_async()
            return False, f"The {self._name} took too long and was cancelled."
        if "result" not in box:
            return False, (
                f"The arm rejected the {self._name}. It is busy, or it was stopped "
                "and must be resumed."
            )
        result = box["result"].result
        if not result.success:
            return False, result.message or f"The {self._name} failed."
        return True, result.message or f"The {self._name} succeeded."
