#!/usr/bin/env python3
"""ROS 2 node: /speech_text -> Claude -> llm_response.

Subscribes:  speech_text   (std_msgs/String)  -- recognized speech
Publishes:   llm_response  (std_msgs/String)  -- text for colman_audio_output to speak
Actions:     pick, place   (colman_interfaces) -- stack_cube and pick_and_place
             move_to       (colman_interfaces) -- go_to_pose
Services:    stop, resume  (std_srvs/Trigger)  -- spoken stop and the stop/resume tools
             release       (std_srvs/Trigger)  -- release_cube

Requires ANTHROPIC_API_KEY, from the environment or from the repository's .env
file (an exported variable wins). The node exits at startup without it.
The subscription callback only enqueues. A single worker thread handles one
utterance at a time.
"""

import os
import queue
import threading
import time
import traceback

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from colman_interfaces.action import MoveTo, Pick, Place
from colman_interfaces.msg import WorldState
from colman_llm.action_call import ActionCall
from colman_llm.claude_client import ClaudeClient
from colman_llm.env_file import load_env_file
from colman_llm.executor import FakeExecutor, PickAndPlaceExecutor
from colman_llm.service_call import ServiceCall
from colman_llm.stop_words import is_stop_command
from colman_llm.world_view import NAME_OF, WorldView
from colman_llm.world_watch import WorldWatch

QUEUE_SIZE = 10
REPEAT_STOP_S = (
    3.0  # a stop said again within this, still stopped, is not acted on twice
)
WORLD_STALE_S = 2.0  # older than this, the world_state is not trusted
WATCH_PERIOD_S = 0.5  # how often the table is checked for changes
ERROR_LINE = "Sorry, something went wrong with that."


class LlmNode(Node):
    def __init__(self):
        super().__init__("llm_node")

        self.declare_parameter("model", "claude-sonnet-5-5")
        # Say so, unprompted, when the table changes by itself (a cube lost or moved).
        self.declare_parameter("announce_changes", True)
        self.declare_parameter("pick_timeout_s", 120.0)
        self.declare_parameter("place_timeout_s", 120.0)
        self.declare_parameter("move_timeout_s", 60.0)
        # dry_run answers tool calls with canned results and never moves the arm.
        self.declare_parameter("dry_run", False)

        load_env_file()
        if not os.getenv("ANTHROPIC_API_KEY"):
            raise RuntimeError(
                "Set ANTHROPIC_API_KEY in your environment or in the .env file."
            )

        self._world_msg = None
        self._world_at = 0.0
        self._world_seen = None  # the last table description, to log changes
        self._claude = ClaudeClient(
            self.get_parameter("model").value,
            self.get_logger(),
            world=self._world_text,
        )
        self._pick = self._place = self._move = self._stop_call = None
        self._last_stop_s = -1e9  # when a spoken stop was last acted on
        self._stop_generation = 0  # bumped by a spoken stop; drops in-flight replies
        self._executor = self._make_executor()
        self._pub = self.create_publisher(String, "llm_response", 10)

        self._working = False  # the worker is handling an utterance
        self._watch = WorldWatch(lambda tag: NAME_OF.get(tag, "an unidentified cube"))
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
        self.create_subscription(WorldState, "/world_state", self._on_world, 10)
        if self.get_parameter("announce_changes").value:
            self.create_timer(WATCH_PERIOD_S, self._announce_changes)
        self.get_logger().info("Listening on speech_text.")

    def _on_world(self, msg):
        self._world_msg = msg
        self._world_at = time.monotonic()
        text = None if msg.stale else WorldView.from_msg(msg).status()
        if text != self._world_seen:
            self._world_seen = text
            self.get_logger().info(f"World changed: {text or 'no view'}")

    def _announce_changes(self):
        """Speak up when the table changes by itself. Quiet while the arm is working."""
        busy = self._working or not self._queue.empty() or self._executor.busy
        said = self._watch.update(self._world(), time.monotonic(), busy)
        if said:
            text = " ".join(said)
            self.get_logger().info(f"Announcing: {text}")
            self._pub.publish(String(data=text))
            if not self._working:
                self._claude.note_spoken(text)

    def _world_text(self):
        """The table as Claude is told about it in every request, or None."""
        view = self._world()
        return None if view is None else view.status()

    def _world(self):
        """The latest world_state as a WorldView, or None if it is missing or stale."""
        msg = self._world_msg
        if msg is None or msg.stale:
            return None
        if time.monotonic() - self._world_at > WORLD_STALE_S:
            return None
        return WorldView.from_msg(msg)

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
        release = ServiceCall(
            self.create_client(Trigger, "release"), Trigger.Request(), "release", log
        )
        move = self._move = ActionCall(
            ActionClient(self, MoveTo, "move_to"), "move", log
        )
        pick_s = self.get_parameter("pick_timeout_s").value
        place_s = self.get_parameter("place_timeout_s").value
        move_s = self.get_parameter("move_timeout_s").value
        return PickAndPlaceExecutor(
            lambda frame: pick.call(Pick.Goal(frame=frame), pick_s),
            lambda frame: place.call(Place.Goal(location=frame), place_s),
            log,
            self._world,
            stop=stop.call,
            resume=resume.call,
            release=release.call,
            move_to=lambda pose: move.call(MoveTo.Goal(pose_name=pose), move_s),
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
        now = time.monotonic()
        if self._executor.stopped and now - self._last_stop_s < REPEAT_STOP_S:
            return  # said again while the first stop is still being acknowledged
        self._last_stop_s = now
        self._stop_generation += 1
        self._executor.set_stopped(True)
        if self._stop_call is not None:
            self._stop_call.call_async()  # the server rejects goals until /resume
            self._pick.cancel_active()
            self._place.cancel_active()
            self._move.cancel_active()
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
            self._working = True
            try:
                reply = self._claude.say(
                    text,
                    self._executor.run_tool,
                    lambda words: self._speak(words, generation),
                )
            except Exception:  # noqa: BLE001 - one bad tool call must not end the worker
                self.get_logger().error(f"Failed on {text!r}\n{traceback.format_exc()}")
                reply = ERROR_LINE
            finally:
                self._working = False
            if generation != self._stop_generation:
                self.get_logger().info("Stopped while working: reply dropped.")
            elif reply:
                self._pub.publish(String(data=reply))
                self.get_logger().info(f"Replied: {reply}")

    def _speak(self, words, generation):
        """Say what Claude wrote before a tool call, unless a stop came since."""
        if generation == self._stop_generation:
            self._pub.publish(String(data=words))
            self.get_logger().info(f"Said: {words}")

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
