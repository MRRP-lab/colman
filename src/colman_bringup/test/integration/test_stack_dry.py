"""Offline smoke test of the voice stack: stack.launch.py with scripted Claude replies.

Launches the real stack launch file with the parts that need a camera, a microphone, a
speaker, a display or the arm left out (`perception`, `audio` and `motion` false).
Claude is replaced by the scripted replies in claude_script.json (COLMAN_LLM_SCRIPT), so
no ANTHROPIC_API_KEY or network is needed. The test publishes on /speech_text and checks
/llm_response and the tool calls that reach the executor (dry_run stack), then, with
dry_run off, checks the spoken stop against a fake motion server in this process: /stop
is called, the active goal is cancelled and nothing queued behind it runs.

Run it with `pixi run test-stack`. Everything runs in its own ROS domain with DDS
limited
to localhost.
"""

import ast
import os
import random
import re
import signal
import subprocess
import threading
import time
from pathlib import Path

import pytest

HERE = Path(__file__).parent
SCRIPT = HERE / "claude_script.json"
UP_TIMEOUT_S = 60
REPLY_TIMEOUT_S = 15


def wait_for(condition, timeout_s, what):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = condition()
        if value:
            return value
        time.sleep(0.05)
    raise AssertionError(f"Timed out after {timeout_s}s waiting for {what}")


class LaunchedStack:
    """`ros2 launch colman_bringup stack.launch.py` in its own process group."""

    def __init__(self, log_path, domain_id, *launch_args):
        self.log_path = log_path
        self.domain_id = domain_id
        self._log = open(log_path, "w")
        self.proc = subprocess.Popen(
            [
                "ros2",
                "launch",
                "colman_bringup",
                "stack.launch.py",
                "driver:=false",
                "perception:=false",
                "motion:=false",
                "audio:=false",
                *launch_args,
            ],
            env=isolated_env(domain_id),
            stdout=self._log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def log(self):
        return self.log_path.read_text(errors="replace")

    def stop(self):
        for sig, wait_s in ((signal.SIGINT, 20), (signal.SIGKILL, 5)):
            if self.proc.poll() is None:
                os.killpg(self.proc.pid, sig)
            deadline = time.monotonic() + wait_s
            while time.monotonic() < deadline and self.proc.poll() is None:
                time.sleep(0.1)
        self._log.close()


class Harness:
    """The test's own ROS node: speaks to the stack and listens to its replies."""

    def __init__(self, domain_id):
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from std_msgs.msg import String

        self._String = String
        rclpy.init(domain_id=domain_id)
        self.node = rclpy.create_node("stack_test")
        self.replies = []
        self._lock = threading.Lock()
        self.node.create_subscription(String, "llm_response", self._on_reply, 10)
        self.speech = self.node.create_publisher(String, "speech_text", 10)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _on_reply(self, msg):
        with self._lock:
            self.replies.append(msg.data)

    def wait_for_llm_node(self):
        wait_for(
            lambda: (
                "llm_node" in self.node.get_node_names()
                and self.speech.get_subscription_count() > 0
                and self.node.get_publishers_info_by_topic("llm_response")
            ),
            UP_TIMEOUT_S,
            "llm_node to come up",
        )

    def say(self, text):
        self.speech.publish(self._String(data=text))

    def reply_count(self):
        with self._lock:
            return len(self.replies)

    def wait_reply(self, text, since=0):
        """Wait for a reply containing text, among those after the first `since`."""

        def seen():
            with self._lock:
                return any(text in r for r in self.replies[since:])

        wait_for(seen, REPLY_TIMEOUT_S, f"reply {text!r}; got {self.replies}")

    def close(self):
        import rclpy

        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()
        self.thread.join(timeout=5)


def isolated_env(domain_id):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["COLMAN_LLM_SCRIPT"] = str(SCRIPT)
    env.pop("ANTHROPIC_API_KEY", None)  # prove the scripted path needs no key
    return env


@pytest.fixture(scope="module")
def domain_ids():
    """One ROS domain per stack, so the two stacks never hear each other."""
    first = random.randint(100, 190)
    return {"dry": first, "live": first + 1}


@pytest.fixture(scope="module")
def dry_stack(domain_ids, tmp_path_factory):
    stack = LaunchedStack(
        tmp_path_factory.mktemp("stack_dry_logs") / "stack_dry.log",
        domain_ids["dry"],
        "dry_run:=true",
    )
    yield stack
    stack.stop()


def tool_calls(stack):
    """(name, args, ok, message) of every tool call the LLM node logged, in order."""
    pattern = r"\btool (\w+) (\{.*?\}) -> ok=(True|False): (.*)"
    return [
        (name, ast.literal_eval(args), ok, message)
        for name, args, ok, message in re.findall(pattern, stack.log())
    ]


def tool_result(stack, name, since=0):
    found = [c for c in tool_calls(stack)[since:] if c[0] == name]
    return found[-1] if found else None


def run_tool(stack, harness, utterance, tool):
    """Say utterance and return the logged (name, args, ok, message) of its call."""
    seen = len(tool_calls(stack))
    harness.say(utterance)
    return wait_for(
        lambda: tool_result(stack, tool, seen), REPLY_TIMEOUT_S, f"tool {tool}"
    )


def assert_alive(stack):
    assert stack.proc.poll() is None, f"launch exited:\n{stack.log()}"
    assert "process has died" not in stack.log(), stack.log()


def test_dry_stack_comes_up(dry_stack, dry_harness):
    dry_harness.wait_for_llm_node()
    assert_alive(dry_stack)
    assert "dry_run: tool calls get canned results" in dry_stack.log()


def test_speech_gets_a_reply(dry_stack, dry_harness):
    dry_harness.wait_for_llm_node()
    start = dry_harness.reply_count()
    dry_harness.say("hello robot")
    dry_harness.wait_reply("Hello there.", start)
    assert_alive(dry_stack)


@pytest.mark.parametrize(
    "utterance, tool, args, reply, fake_lines",
    [
        (
            "stack the blue cubes",
            "stack_cube",
            {"color": "blue", "count": 2},
            "Stacked the blue cubes.",
            ["FAKE pick tag_6", "FAKE place tag_5", "FAKE pick tag_7"],
        ),
        (
            "put the cube on top, white cube two",
            "pick_and_place",
            {"object": "white cube 2", "destination": "white cube 3"},
            "Moved the white cube.",
            ["FAKE pick tag_2", "FAKE place tag_3"],
        ),
        (
            "go home please",
            "go_to_pose",
            {"pose": "home"},
            "At home.",
            ["FAKE move_to home"],
        ),
        ("let go of it", "release_cube", {}, "Released.", []),
        ("what is your state", "report_state", {}, "Here is the state.", []),
    ],
)
def test_tool_calls_reach_the_fake_executor(
    dry_stack, dry_harness, utterance, tool, args, reply, fake_lines
):
    dry_harness.wait_for_llm_node()
    start = dry_harness.reply_count()
    _, got_args, ok, message = run_tool(dry_stack, dry_harness, utterance, tool)
    assert (got_args, ok) == (args, "True"), message
    dry_harness.wait_reply(reply, start)
    if tool in ("stack_cube", "pick_and_place", "go_to_pose"):
        dry_harness.wait_reply("On it.", start)
    log = dry_stack.log()
    for line in fake_lines:
        assert line in log
    assert_alive(dry_stack)


def test_stop_word_then_resume(dry_stack, dry_harness):
    dry_harness.wait_for_llm_node()
    start = dry_harness.reply_count()
    dry_harness.say("stop right now")
    dry_harness.wait_reply("Stopped.", start)
    assert "Stop word heard: stop right now" in dry_stack.log()

    # The arm reports being stopped until it is resumed.
    message = run_tool(dry_stack, dry_harness, "what is your state", "report_state")[3]
    assert "The arm is stopped." in message

    _, _, ok, message = run_tool(
        dry_stack, dry_harness, "okay carry on", "resume_robot"
    )
    assert ok == "True" and "can move again" in message

    message = run_tool(dry_stack, dry_harness, "what is your state", "report_state")[3]
    assert "The arm is ready." in message


def test_pause_goes_through_the_stop_tool(dry_stack, dry_harness):
    """A stop phrased without a stop word goes through the stop_robot tool."""
    dry_harness.wait_for_llm_node()
    _, _, ok, message = run_tool(
        dry_stack, dry_harness, "please pause the arm", "stop_robot"
    )
    assert ok == "True" and "stopped" in message
    _, _, ok, _ = run_tool(dry_stack, dry_harness, "carry on", "resume_robot")
    assert ok == "True"


# --- Second stack: dry_run off, against a fake motion server in this process --------


class FakeMotion:
    """Stands in for the motion server: /pick, /place, /move_to, /stop, /resume."""

    def __init__(self, node):
        from rclpy.action import ActionServer, CancelResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from std_srvs.srv import Trigger

        from colman_interfaces.action import MoveTo, Pick, Place

        self.events = []  # ("pick", frame), ("stop",), ("canceled", "pick"), ...
        self.stopped = False
        self.pick_started = threading.Event()
        self.finish_picks = threading.Event()
        self._lock = threading.Lock()
        group = ReentrantCallbackGroup()
        self.servers = [
            ActionServer(
                node,
                action,
                name,
                self._run(name, action, field),
                goal_callback=self._on_goal,
                cancel_callback=lambda goal: CancelResponse.ACCEPT,
                callback_group=group,
            )
            for name, action, field in (
                ("pick", Pick, "frame"),
                ("place", Place, "location"),
                ("move_to", MoveTo, "pose_name"),
            )
        ]
        for name, handler in (
            ("stop", self._stop),
            ("resume", self._resume),
            ("release", self._release),
        ):
            node.create_service(Trigger, name, handler, callback_group=group)

    def record(self, *event):
        with self._lock:
            self.events.append(event)

    def has(self, *event):
        with self._lock:
            return event in self.events

    def count(self, kind):
        with self._lock:
            return sum(1 for e in self.events if e[0] == kind)

    def _on_goal(self, goal):
        from rclpy.action import GoalResponse

        if self.stopped:
            self.record("rejected")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _run(self, kind, action, field):
        def execute(goal_handle):
            self.record(kind, getattr(goal_handle.request, field))
            if kind == "pick":
                self.pick_started.set()
                # A pick lasts until it is cancelled or the test lets it finish.
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    if goal_handle.is_cancel_requested:
                        self.record("canceled", kind)
                        goal_handle.canceled()
                        return action.Result()
                    if self.finish_picks.is_set():
                        break
                    time.sleep(0.02)
            goal_handle.succeed()
            result = action.Result()
            result.success = True
            return result

        return execute

    def _stop(self, request, response):
        self.stopped = True
        self.record("stop")
        response.success = True
        return response

    def _resume(self, request, response):
        self.stopped = False
        self.record("resume")
        response.success = True
        return response

    def _release(self, request, response):
        self.record("release")
        response.success = True
        return response


@pytest.fixture
def dry_harness(dry_stack):
    h = Harness(dry_stack.domain_id)
    yield h
    h.close()


@pytest.fixture
def live_harness(live_stack):
    h = Harness(live_stack.domain_id)
    yield h
    h.close()


@pytest.fixture(scope="module")
def live_stack(domain_ids, tmp_path_factory):
    stack = LaunchedStack(
        tmp_path_factory.mktemp("stack_live_logs") / "stack_live.log",
        domain_ids["live"],
        "dry_run:=false",
    )
    yield stack
    stack.stop()


def test_spoken_stop_cancels_the_goal_before_anything_queued(live_stack, live_harness):
    live_harness.wait_for_llm_node()
    motion = FakeMotion(live_harness.node)  # served by the harness's spinning executor
    # The LLM node's action clients are discovered before it sends anything.
    wait_for(
        lambda: all(
            live_harness.node.count_subscribers(f"/{name}/_action/feedback") > 0
            for name in ("pick", "place", "move_to")
        ),
        UP_TIMEOUT_S,
        "the LLM node to see the fake motion server",
    )

    start = live_harness.reply_count()
    live_harness.say("stack the blue cubes")  # blocks in /pick
    wait_for(motion.pick_started.is_set, REPLY_TIMEOUT_S, "the pick goal")
    live_harness.wait_reply("On it.", start)
    live_harness.say("go home")  # queued behind the running job
    live_harness.say("freeze")  # a stop word jumps the queue

    live_harness.wait_reply("Stopped.", start)
    wait_for(lambda: motion.has("stop"), REPLY_TIMEOUT_S, "/stop to be called")
    wait_for(
        lambda: motion.has("canceled", "pick"), REPLY_TIMEOUT_S, "pick to be cancelled"
    )
    wait_for(
        lambda: "Stopped while working" in live_stack.log(),
        REPLY_TIMEOUT_S,
        "the in-flight reply to be dropped",
    )
    # The queued "go home" never reached the arm, and the cancelled stack_cube's own
    # reply was dropped.
    assert "Dropped 1 queued utterances." in live_stack.log()
    assert motion.count("move_to") == 0
    assert "Stacked the blue cubes." not in live_harness.replies
    assert_alive(live_stack)

    # The server now rejects goals, and the tool says so, until /resume.
    result = run_tool(
        live_stack, live_harness, "put the white cube two there", "pick_and_place"
    )
    assert result[2] == "False" and motion.has("rejected")

    _, _, ok, _ = run_tool(live_stack, live_harness, "carry on", "resume_robot")
    assert ok == "True" and motion.has("resume")

    # After the resume the same request is accepted and completes.
    motion.finish_picks.set()
    result = run_tool(
        live_stack, live_harness, "put the white cube two there", "pick_and_place"
    )
    assert result[2] == "True", result
    assert motion.has("pick", "tag_2") and motion.has("place", "tag_3")
    assert_alive(live_stack)
