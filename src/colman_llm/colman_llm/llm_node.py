#!/usr/bin/env python3
"""ROS 2 node: /speech_text -> Claude -> llm_response.

Subscribes:  speech_text   (std_msgs/String)  -- recognized speech
Publishes:   llm_response  (std_msgs/String)  -- text for colman_audio_output to speak
Actions:     pick, place   (colman_interfaces) -- the pick_and_place tool
Services:    stop, resume  (std_srvs/Trigger)  -- spoken stop and the stop/resume tools

Requires ANTHROPIC_API_KEY, from the environment or from the repository's .env
file (an exported variable wins). The node exits at startup without it.
The subscription callback only enqueues. A single worker thread handles one
utterance at a time.
"""

import os
import queue
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from colman_interfaces.action import Pick, Place
from colman_llm.action_call import ActionCall
from colman_llm.claude_client import ClaudeClient
from colman_llm.env_file import load_env_file
from colman_llm.executor import FakeExecutor, PickAndPlaceExecutor
from colman_llm.service_call import ServiceCall
from colman_llm.stop_words import is_stop_command

QUEUE_SIZE = 10


class LlmNode(Node):
    def __init__(self):
        super().__init__("llm_node")

        self.declare_parameter("model", "claude-sonnet-5-5")
        self.declare_parameter("pick_timeout_s", 120.0)
        self.declare_parameter("place_timeout_s", 120.0)
        # dry_run answers tool calls with canned results and never moves the arm.
        self.declare_parameter("dry_run", False)

        load_env_file()
        if not os.getenv("ANTHROPIC_API_KEY"):
            raise RuntimeError(
                "Set ANTHROPIC_API_KEY in your environment or in the .env file."
            )

        self._claude = ClaudeClient(
            self.get_parameter("model").value, self.get_logger()
        )
        self._pick = self._place = self._stop_call = None
        self._stop_generation = 0  # bumped by a spoken stop; drops in-flight replies
        self._executor = self._make_executor()
        self._pub = self.create_publisher(String, "llm_response", 10)

        self._queue = queue.Queue(maxsize=QUEUE_SIZE)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._work, name="llm", daemon=True)
        self._worker.start()

        self.create_subscription(
            String,
            "speech_text",
            self._on_speech,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Listening on speech_text.")

    def _make_executor(self):
        log = self.get_logger()
        if self.get_parameter("dry_run").value:
            log.warn(
                "dry_run: tool calls get canned results and the arm will not move."
            )
            return FakeExecutor(log)
        pick = self._pick = ActionCall(ActionClient(self, Pick, "pick"), "pick", log)
        place = self._place = ActionCall(
            ActionClient(self, Place, "place"), "place", log
        )
        stop = self._stop_call = ServiceCall(
            self.create_client(Trigger, "stop"), Trigger.Request(), "stop", log
        )
        resume = ServiceCall(
            self.create_client(Trigger, "resume"), Trigger.Request(), "resume", log
        )
        pick_s = self.get_parameter("pick_timeout_s").value
        place_s = self.get_parameter("place_timeout_s").value
        return PickAndPlaceExecutor(
            lambda frame: pick.call(Pick.Goal(frame=frame), pick_s),
            lambda frame: place.call(Place.Goal(location=frame), place_s),
            log,
            stop=stop.call,
            resume=resume.call,
        )

    def _on_speech(self, msg):
        # A spoken stop must not wait behind the pick and place it is meant to stop.
        if is_stop_command(msg.data):
            self._emergency_stop(msg.data)
            return
        try:
            self._queue.put_nowait(msg.data)
        except queue.Full:
            self.get_logger().warn(f"Queue full, dropped: {msg.data}")

    def _emergency_stop(self, text):
        log = self.get_logger()
        log.warn(f"Stop word heard: {text}")
        self._stop_generation += 1
        if self._stop_call is not None:
            self._stop_call.call_async()  # the server rejects goals until /resume
            self._pick.cancel_active()
            self._place.cancel_active()
        dropped = 0
        while True:
            try:
                self._queue.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        if dropped:
            log.info(f"Dropped {dropped} queued utterances.")
        self._pub.publish(String(data="Stopped."))

    def _work(self):
        while not self._stop.is_set():
            try:
                text = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self.get_logger().info(f"Heard: {text}")
            generation = self._stop_generation
            reply = self._claude.say(text, self._run_tool)
            if generation != self._stop_generation:
                self.get_logger().info("Stopped while working: reply dropped.")
            elif reply:
                self._pub.publish(String(data=reply))
                self.get_logger().info(f"Replied: {reply}")

    def _run_tool(self, name, args):
        # Tell the user the command was heard before a long action starts.
        if name == "pick_and_place":
            self._pub.publish(String(data="On it."))
        return self._executor.run_tool(name, args)

    def destroy_node(self):
        self._stop.set()
        self._worker.join(timeout=5.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = LlmNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
