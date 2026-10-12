import signal
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from colman_interfaces.action import MoveTo, Pick, Place
from colman_motion.patterns import PATTERNS
from colman_motion.tag_lookup import TagLookup

TAG_PATTERN = ["tag_1", "tag_3", "tag_5", "tag_6"]
DEFAULT_PATTERN = "left_to_right"
LOOK_POSE = "Look3"
SERVER_WAIT_S = 10.0
CANCEL_WAIT_S = 2.0
SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in SIGNALS:
        signal.signal(sig, signal.default_int_handler)
    node = Node("pick_and_place")
    tag = TagLookup()
    logger = node.get_logger()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(tag)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    move_client = ActionClient(node, MoveTo, "move_to")
    pick_client = ActionClient(node, Pick, "pick")
    place_client = ActionClient(node, Place, "place")

    def feedback_cb(msg):
        logger.info(f"stage: {msg.feedback.stage}")

    def wait(future, timeout_s):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done()

    def cancel(handle):
        logger.warn("Canceling goal")
        wait(handle.cancel_goal_async(), CANCEL_WAIT_S)

    sent = None

    def run_goal(client, goal, feedback_cb=None):
        nonlocal sent
        sent = client.send_goal_async(goal, feedback_callback=feedback_cb)
        if not wait(sent, SERVER_WAIT_S):
            logger.error("No response to goal")
            return None
        handle = sent.result()
        if not handle.accepted:
            sent = None
            return None
        result_future = handle.get_result_async()
        while not result_future.done():
            if not client.server_is_ready():
                logger.error("Lost action server")
                sent = None
                return None
            time.sleep(0.05)
        sent = None
        return result_future.result()

    try:
        args = remove_ros_args(sys.argv)
        pattern = args[1] if len(args) > 1 else DEFAULT_PATTERN
        if pattern not in PATTERNS:
            logger.error(f"Unknown pattern {pattern}")
            return

        for client in (move_client, pick_client, place_client):
            if not client.wait_for_server(timeout_sec=SERVER_WAIT_S):
                logger.error("Action servers not available")
                return

        response = run_goal(move_client, MoveTo.Goal(pose_name=LOOK_POSE))
        if response is None or not response.result.success:
            logger.error("Look move failed")
            return

        tags = {}
        for tag_frame in TAG_PATTERN:
            tag_pose = tag.get_tag_pose(tag_frame)
            if tag_pose is not None:
                translation = tag_pose.translation
                tags[tag_frame] = (translation.x, translation.y, translation.z)
        logger.info(f"Visible tags: {sorted(tags)}")

        if len(tags) < 2:
            logger.error("Need at least two tags to stack")
            return

        order = PATTERNS[pattern](tags)
        logger.info(f"Pattern {pattern}: {order}")

        previous = order[0]
        for tag_frame in order[1:]:
            response = run_goal(pick_client, Pick.Goal(frame=tag_frame), feedback_cb)
            if response is None:
                logger.error("Pick rejected")
                return
            if response.status == GoalStatus.STATUS_CANCELED:
                logger.error("Pick canceled")
                return
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                if response.result.message.endswith("not visible"):
                    logger.warn(f"Skipping {tag_frame}: {response.result.message}")
                    continue
                logger.error(f"Pick failed: {response.result.message}")
                return

            response = run_goal(
                place_client, Place.Goal(location=previous), feedback_cb
            )
            if response is None:
                logger.error("Place rejected")
                return
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                logger.error(f"Place failed: {response.result.message}")
                return

            previous = tag_frame

        logger.info("Stacking complete")
    except KeyboardInterrupt:
        for sig in SIGNALS:
            signal.signal(sig, signal.SIG_IGN)
        if sent is not None and wait(sent, CANCEL_WAIT_S) and sent.result().accepted:
            cancel(sent.result())
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
