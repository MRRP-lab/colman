#!/usr/bin/env python3
"""ROS 2 node: /speech_text -> Claude -> llm_response.

Subscribes:  speech_text   (std_msgs/String)  -- recognized speech
Publishes:   llm_response  (std_msgs/String)  -- text for colman_audio_output to speak

Requires ANTHROPIC_API_KEY, from the environment or from the repository's .env
file (an exported variable wins). The node exits at startup without it.
The subscription callback only enqueues. A single worker thread handles one
utterance at a time.
"""

import os
import queue
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from colman_llm.claude_client import ClaudeClient
from colman_llm.env_file import load_env_file
from colman_llm.executor import FakeExecutor

QUEUE_SIZE = 10


class LlmNode(Node):
    def __init__(self):
        super().__init__("llm_node")

        self.declare_parameter("model", "claude-sonnet-5-5")

        load_env_file()
        if not os.getenv("ANTHROPIC_API_KEY"):
            raise RuntimeError(
                "Set ANTHROPIC_API_KEY in your environment or in the .env file."
            )

        self._claude = ClaudeClient(
            self.get_parameter("model").value, self.get_logger()
        )
        self._executor = FakeExecutor(self.get_logger())
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

    def _on_speech(self, msg):
        try:
            self._queue.put_nowait(msg.data)
        except queue.Full:
            self.get_logger().warn(f"Queue full, dropped: {msg.data}")

    def _work(self):
        while not self._stop.is_set():
            try:
                text = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self.get_logger().info(f"Heard: {text}")
            reply = self._claude.say(text, self._run_tool)
            if reply:
                self._pub.publish(String(data=reply))
                self.get_logger().info(f"Replied: {reply}")

    def _run_tool(self, name, args):
        # Tell the user the command was heard before a long action starts.
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
