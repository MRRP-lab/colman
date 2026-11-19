import math
import signal
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from colman_interfaces.action import MovePose

EE_DOWN = (1.0, 0.0, 0.0, 0.0)

CIRCLE_CENTER = (0.25, 0.0, 0.2)

CIRCLE_RADIUS = 0.08

CIRCLE_POINTS = 12

SERVER_WAIT_S = 10.0
CANCEL_WAIT_S = 2.0
SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def circle_waypoints():
    cx, cy, cz = CIRCLE_CENTER
    for i in range(CIRCLE_POINTS):
        angle = 2 * math.pi * i / CIRCLE_POINTS
        yield (
            cx + CIRCLE_RADIUS * math.cos(angle),
            cy + CIRCLE_RADIUS * math.sin(angle),
            cz,
        )


def make_goal(x, y, z):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = EE_DOWN
    return MovePose.Goal(pose=pose, slow=True)


def wait(future, timeout_s):
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    return future.done()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in SIGNALS:
        signal.signal(sig, signal.default_int_handler)
    node = Node("demo_circle")
    logger = node.get_logger()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    client = ActionClient(node, MovePose, "move_pose")
    sent = None

    try:
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            logger.error("Action server not available")
            return

        while True:
            for waypoint in circle_waypoints():
                sent = client.send_goal_async(make_goal(*waypoint))
                if not wait(sent, SERVER_WAIT_S):
                    logger.error("No response to goal")
                    return
                handle = sent.result()
                if not handle.accepted:
                    logger.warn("Goal rejected, stopping")
                    sent = None
                    return
                result_future = handle.get_result_async()
                while not result_future.done():
                    if not client.server_is_ready():
                        logger.error("Lost action server")
                        sent = None
                        return
                    time.sleep(0.05)
                sent = None
                if result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
                    logger.warn("Stopping")
                    return
    except KeyboardInterrupt:
        for sig in SIGNALS:
            signal.signal(sig, signal.SIG_IGN)
        if sent is not None and wait(sent, CANCEL_WAIT_S) and sent.result().accepted:
            logger.warn("Canceling goal")
            wait(sent.result().cancel_goal_async(), CANCEL_WAIT_S)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
