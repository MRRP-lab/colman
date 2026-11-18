"""Shared machinery for the motion-server integration tests.

Stack     the processes of a test stack, which a test can kill, pause and restart.
Client    an rclpy node, spun in a thread, that sends goals and watches the server.
GoalRun   one goal sent through Client: its feedback stages, state and result.

Everything waits on a condition with a timeout. Nothing here sleeps and hopes.
"""

import json
import os
import random
import signal
import subprocess
import threading
import time
from pathlib import Path

from action_msgs.msg import GoalStatus, GoalStatusArray

HERE = Path(__file__).parent
STARTUP_TIMEOUT_S = 180
GOAL_TIMEOUT_S = 240
CALL_TIMEOUT_S = 15

ACTIVE_STATES = (
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
)
OUTCOMES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_ABORTED: "aborted",
    GoalStatus.STATUS_CANCELED: "canceled",
}
TERMINAL = tuple(OUTCOMES.values())


_domains_used = set()


def isolated_env():
    """The environment for a stack of its own: a fresh ROS domain, localhost only."""
    env = dict(os.environ)
    domain = random.choice([d for d in range(100, 201) if d not in _domains_used])
    _domains_used.add(domain)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_LOCALHOST_ONLY"] = "1"
    return env


def wait_until(condition, timeout_s, what, poll_s=0.05):
    """Poll condition() until it is truthy and return it; TimeoutError names `what`."""
    deadline = time.monotonic() + timeout_s
    while True:
        value = condition()
        if value:
            return value
        if time.monotonic() >= deadline:
            raise TimeoutError(f"timed out after {timeout_s}s waiting for {what}")
        time.sleep(poll_s)


class Stack:
    """The processes of the test stack, each in its own process group."""

    def __init__(self, env, log_dir):
        self.env = env
        self.log_dir = log_dir
        self.procs = {}
        self.commands = {}

    def start(self, name, command):
        self.commands[name] = command
        log = open(self.log_dir / f"{name}.log", "a")
        log.write(f"\n=== start {name} ===\n")
        log.flush()
        self.procs[name] = subprocess.Popen(
            command,
            env=self.env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        return self.procs[name]

    def restart(self, name):
        self.kill(name)
        return self.start(name, self.commands[name])

    def alive(self, name):
        proc = self.procs.get(name)
        return proc is not None and proc.poll() is None

    def signal(self, name, sig):
        proc = self.procs[name]
        if proc.poll() is None:
            os.killpg(proc.pid, sig)

    def kill(self, name):
        """SIGKILL the whole process group and wait for it to be gone."""
        proc = self.procs[name]
        self.signal(name, signal.SIGKILL)
        wait_until(lambda: proc.poll() is not None, 10, f"{name} to die")
        # The group leader (ros2 launch) can go before its children do.
        wait_until(lambda: not self._group_alive(proc.pid), 10, f"{name} children")

    @staticmethod
    def _group_alive(pgid):
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return False
        return True

    def pause(self, name):
        self.signal(name, signal.SIGSTOP)

    def resume(self, name):
        self.signal(name, signal.SIGCONT)

    def stop(self):
        for name in self.procs:
            if self.alive(name):
                self.signal(name, signal.SIGCONT)
        for sig, wait_s in ((signal.SIGINT, 20), (signal.SIGKILL, 5)):
            for name in self.procs:
                if self._group_alive(self.procs[name].pid):
                    self.signal(name, sig)
            deadline = time.monotonic() + wait_s
            while time.monotonic() < deadline and any(
                self._group_alive(p.pid) for p in self.procs.values()
            ):
                time.sleep(0.2)


class GoalRun:
    """One goal: what the server sent back for it, as it happened."""

    def __init__(self, client, label, goal):
        self.client = client
        self.label = label
        self.goal = goal
        self.cond = threading.Condition()
        self.accepted = None  # None until the server has answered
        self.handle = None
        self.stages = []  # (monotonic time, stage) for each feedback
        self.status = None
        self.result = None
        self.sent_at = time.monotonic()
        self.done_at = None

    def _on_feedback(self, msg):
        with self.cond:
            self.stages.append((time.monotonic(), msg.feedback.stage))
            self.cond.notify_all()

    def _on_response(self, future):
        handle = future.result()
        with self.cond:
            self.handle = handle
            self.accepted = bool(handle is not None and handle.accepted)
            self.cond.notify_all()
        if self.accepted:
            handle.get_result_async().add_done_callback(self._on_result)
        else:
            with self.cond:
                self.done_at = time.monotonic()
                self.cond.notify_all()

    def _on_result(self, future):
        wrapped = future.result()
        with self.cond:
            self.status = wrapped.status
            self.result = wrapped.result
            self.done_at = time.monotonic()
            self.cond.notify_all()

    def wait(self, predicate, timeout_s, what):
        deadline = time.monotonic() + timeout_s
        with self.cond:
            while not predicate():
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError(f"{self.label}: timed out waiting for {what}")
                self.cond.wait(left)

    def wait_accepted(self, timeout_s=CALL_TIMEOUT_S):
        self.wait(lambda: self.accepted is not None, timeout_s, "the goal response")
        return self.accepted

    def wait_stage(self, stage, timeout_s=60):
        """Wait for feedback `stage`. Returns False if the goal ended first."""
        self.wait(
            lambda: self.done_at is not None or self.saw(stage),
            timeout_s,
            f"stage {stage!r} (saw {self.stage_names()})",
        )
        return self.saw(stage)

    def saw(self, stage):
        return any(s == stage for _, s in self.stages)

    def stage_names(self):
        return [s for _, s in self.stages]

    @property
    def done(self):
        return self.done_at is not None

    def wait_done(self, timeout_s=GOAL_TIMEOUT_S):
        self.wait(lambda: self.done, timeout_s, "a result")
        return self

    @property
    def outcome(self):
        if self.accepted is False:
            return "rejected"
        return OUTCOMES.get(self.status)

    @property
    def message(self):
        return getattr(self.result, "message", "") if self.result else ""

    @property
    def success(self):
        return bool(self.result and self.result.success)

    def cancel(self, timeout_s=CALL_TIMEOUT_S):
        """Ask the server to cancel. Returns the ActionClient cancel return code."""
        self.wait_accepted()
        future = self.handle.cancel_goal_async()
        response = self.client.wait_future(future, timeout_s, f"cancel of {self.label}")
        return response.return_code

    def __repr__(self):
        stages = self.stage_names()
        return f"<{self.label} {self.outcome} {self.message!r} stages={stages}>"


def param_value(value):
    from rcl_interfaces.msg import ParameterType, ParameterValue

    msg = ParameterValue()
    if isinstance(value, bool):
        msg.type, msg.bool_value = ParameterType.PARAMETER_BOOL, value
    elif isinstance(value, int):
        msg.type, msg.integer_value = ParameterType.PARAMETER_INTEGER, value
    elif isinstance(value, float):
        msg.type, msg.double_value = ParameterType.PARAMETER_DOUBLE, value
    elif isinstance(value, str):
        msg.type, msg.string_value = ParameterType.PARAMETER_STRING, value
    else:
        msg.type = ParameterType.PARAMETER_INTEGER_ARRAY
        msg.integer_array_value = list(value)
    return msg


def from_param_value(msg):
    from rcl_interfaces.msg import ParameterType

    return {
        ParameterType.PARAMETER_BOOL: msg.bool_value,
        ParameterType.PARAMETER_INTEGER: msg.integer_value,
        ParameterType.PARAMETER_DOUBLE: msg.double_value,
        ParameterType.PARAMETER_STRING: msg.string_value,
        ParameterType.PARAMETER_INTEGER_ARRAY: list(msg.integer_array_value),
    }[msg.type]


class Client:
    """The test's view of the motion server, from its own ROS context."""

    ACTIONS = ("move_to", "move_pose", "pick", "place")

    def __init__(self, domain_id):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        from colman_interfaces.action import MovePose, MoveTo, Pick, Place

        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        self.node = rclpy.create_node("fault_test_client", context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.types = {"move_to": MoveTo, "move_pose": MovePose, "pick": Pick}
        self.types["place"] = Place
        self.clients = {
            name: ActionClient(self.node, self.types[name], f"/{name}")
            for name in self.ACTIONS
        }
        self.services = {}
        self.lock = threading.Lock()
        # goal uuid -> (status, monotonic time it was first seen in that status)
        self.goal_states = {}
        for name in self.ACTIONS:
            self.node.create_subscription(
                GoalStatusArray,
                f"/{name}/_action/status",
                lambda msg, name=name: self._on_status(name, msg),
                qos_profile_action_status_default,
            )
        self.joints = None  # (monotonic time, {joint: position})
        self.node.create_subscription(
            JointState, "/joint_states", self._on_joints, qos_profile_sensor_data
        )
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def close(self):
        import rclpy

        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def _on_status(self, name, msg):
        now = time.monotonic()
        with self.lock:
            for entry in msg.status_list:
                key = (name, bytes(entry.goal_info.goal_id.uuid))
                if key not in self.goal_states or self.goal_states[key][0] != (
                    entry.status
                ):
                    self.goal_states[key] = (entry.status, now)

    def _on_joints(self, msg):
        self.joints = (time.monotonic(), dict(zip(msg.name, msg.position)))

    # --- waiting on futures ---

    def wait_future(self, future, timeout_s, what):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            raise TimeoutError(f"timed out after {timeout_s}s waiting for {what}")
        return future.result()

    def call(self, service_type, name, request, timeout_s=CALL_TIMEOUT_S):
        key = (service_type, name)
        if key not in self.services:
            self.services[key] = self.node.create_client(service_type, name)
        client = self.services[key]
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"service {name} is not available")
        return self.wait_future(client.call_async(request), timeout_s, f"{name}")

    def trigger(self, name, timeout_s=CALL_TIMEOUT_S):
        from std_srvs.srv import Trigger

        return self.call(Trigger, name, Trigger.Request(), timeout_s)

    # --- parameters of the helper nodes ---

    def set_params(self, target, **values):
        from rcl_interfaces.msg import Parameter
        from rcl_interfaces.srv import SetParameters

        request = SetParameters.Request()
        for name, value in values.items():
            request.parameters.append(Parameter(name=name, value=param_value(value)))
        response = self.call(SetParameters, f"/{target}/set_parameters", request)
        for result in response.results:
            assert result.successful, f"{target}: {result.reason}"

    def get_params(self, target, *names):
        from rcl_interfaces.srv import GetParameters

        request = GetParameters.Request(names=list(names))
        response = self.call(GetParameters, f"/{target}/get_parameters", request)
        values = [from_param_value(v) for v in response.values]
        return values[0] if len(names) == 1 else dict(zip(names, values))

    # --- the fakes ---

    def set_tag(self, number, pose):
        """Put tag_<number> at pose [x, y, z, qx, qy, qz, qw]."""
        overrides = json.loads(self.get_params("fake_tags", "overrides"))
        overrides[str(number)] = list(pose)
        self.set_params("fake_tags", overrides=json.dumps(overrides))

    def hide_tags(self, *numbers):
        self.set_params("fake_tags", hidden=list(numbers) or [0])

    def reset_tags(self):
        self.set_params("fake_tags", hidden=[0], overrides="{}")

    def io_mode(self, mode):
        self.set_params("fake_io", mode=mode)

    def vacuum_on(self):
        return self.get_params("fake_io", "pin0")

    def pause_joint_states(self, paused=True):
        self.set_params("joint_state_relay", paused=paused)

    # --- the motion server ---

    def server_state(self, timeout_s=CALL_TIMEOUT_S):
        """The probe's report: scene, flags, thread count. Raises if unresponsive."""
        return json.loads(self.trigger("/test/state", timeout_s).message)

    def goal(self, kind, **fields):
        """Send a goal of this kind: pick, place, move_to or move_pose."""
        action = self.types[kind]
        goal = action.Goal()
        for key, value in fields.items():
            setattr(goal, key, value)
        run = GoalRun(self, f"{kind}{fields}", goal)
        future = self.clients[kind].send_goal_async(
            goal, feedback_callback=run._on_feedback
        )
        future.add_done_callback(run._on_response)
        return run

    def pick(self, frame):
        return self.goal("pick", frame=frame)

    def place(self, location="", position=None):
        from geometry_msgs.msg import Point

        point = Point(x=position[0], y=position[1], z=position[2]) if position else None
        if point is None:
            return self.goal("place", location=location)
        return self.goal("place", location=location, position=point)

    def move_to(self, pose_name):
        return self.goal("move_to", pose_name=pose_name)

    def move_pose(self, xyz, quat=(1.0, 0.0, 0.0, 0.0), slow=False):
        from geometry_msgs.msg import Pose

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.x, pose.orientation.y, pose.orientation.z = quat[:3]
        pose.orientation.w = quat[3]
        return self.goal("move_pose", pose=pose, slow=slow)

    def active_goals(self):
        """Goals the server currently holds in ACCEPTED, EXECUTING or CANCELING.

        A server that went away took its goals with it, and a restarted one does not
        republish the old ones' end, so goals of an action with no server are dropped.
        """
        gone = [n for n, c in self.clients.items() if not c.server_is_ready()]
        with self.lock:
            for key in [k for k in self.goal_states if k[0] in gone]:
                del self.goal_states[key]
            return {k: v for k, v in self.goal_states.items() if v[0] in ACTIVE_STATES}

    def longest_active_s(self):
        now = time.monotonic()
        active = self.active_goals()
        return max((now - since for _, since in active.values()), default=0.0)

    # --- the arm ---

    def joint_positions(self):
        return None if self.joints is None else self.joints[1]

    def arm_is_moving(self, window_s=0.3, eps=2e-3):
        """True if any joint moved more than eps rad over the next window_s."""
        first = self.joint_positions()
        if first is None:
            return False
        time.sleep(window_s)
        second = self.joint_positions()
        return any(abs(second[j] - first[j]) > eps for j in first if j in second)

    def wait_arm_moving(self, timeout_s=30):
        return wait_until(self.arm_is_moving, timeout_s, "the arm to move", poll_s=0)

    def wait_arm_still(self, timeout_s=15):
        return wait_until(
            lambda: not self.arm_is_moving(0.5), timeout_s, "the arm to stop", poll_s=0
        )
