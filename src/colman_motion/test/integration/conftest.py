"""Fixtures for the fault-injection tests: one stack for the whole session.

The stack is the mock UR driver, fake tags, a fake vacuum IO service, a joint-state
relay and the real motion server (see probe_motion_server.py). The tests break parts
of it on purpose; `healthy` puts it all back before each test and checks the server
came through the last one intact.

Marks (registered in pyproject.toml):
  fault_injection  kills, pauses or restarts a process; minutes, run on demand
  slow             long randomized runs and the soak
Anything unmarked is the fast, deterministic subset that CI runs.
"""

import os
import sys
import time
from pathlib import Path

import pytest
import yaml
from harness import (
    HERE,
    STARTUP_TIMEOUT_S,
    Client,
    Stack,
    isolated_env,
    wait_until,
)

SERVER_STARTUP_S = 120
RECOVERY_S = 90
HOME_TIMEOUT_S = 60
# No goal legitimately stays active this long (a pick takes well under a minute).
GOAL_STUCK_S = 150
ACTIVE_GRACE_S = 1.5
# The motion server's own launch file passes these; keep them in step with
# colman_motion/motion_server.launch.py.
GRIPPER_TYPE = "vacuum"
USE_CAMERA = "true"


def server_params(path):
    """Write the motion server's ROS parameters to `path`, as its launch file does."""
    from moveit_configs_utils import MoveItConfigsBuilder

    mappings = {"gripper_type": GRIPPER_TYPE, "use_camera": USE_CAMERA}
    moveit_config = (
        MoveItConfigsBuilder(robot_name="colman", package_name="colman_moveit_config")
        .robot_description(
            file_path="config/colman_moveit.urdf.xacro", mappings=mappings
        )
        .robot_description_semantic(
            file_path="srdf/colman.srdf.xacro", mappings=mappings
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .moveit_cpp(file_path=_moveit_py_yaml())
        .to_moveit_configs()
    )
    params = moveit_config.to_dict()
    params["use_sim_time"] = False
    # The joint-state relay sits between the driver and the motion server.
    params["planning_scene_monitor_options"]["joint_state_topic"] = "/ms/joint_states"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))


def _moveit_py_yaml():
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("colman_moveit_config"), "config", "moveit_py.yaml"
    )


def motion_server_command(params_file):
    return [
        sys.executable, str(HERE / "probe_motion_server.py"),
        "--ros-args", "--params-file", str(params_file),
        "-r", "/io_and_status_controller/set_io:=/test_io/set_io",
        "-r", "/joint_states:=/ms/joint_states",
    ]  # fmt: skip


def driver_command():
    from ament_index_python.packages import get_package_prefix

    rsp = Path(get_package_prefix("colman_bringup")) / "share/colman_bringup/launch"
    return [
        "ros2", "launch", "ur_robot_driver", "ur_control.launch.py",
        "ur_type:=ur3e", "robot_ip:=0.0.0.0", "use_mock_hardware:=true",
        "launch_dashboard_client:=false", "launch_rviz:=false",
        f"description_launchfile:={rsp / 'rsp.launch.py'}",
    ]  # fmt: skip


class Rig:
    """The stack plus the client, with the operations the tests share."""

    def __init__(self, stack, client):
        self.stack = stack
        self.client = client
        self.baseline = None

    # --- bringing things up ---

    def wait_for_driver(self):
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        controller = ActionClient(
            self.client.node,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )
        wait_until(
            lambda: controller.wait_for_server(timeout_sec=0.5) and self.client.joints,
            STARTUP_TIMEOUT_S,
            "the mock driver's trajectory controller and joint states",
        )
        controller.destroy()

    def wait_for_server(self, timeout_s=SERVER_STARTUP_S):
        wait_until(
            lambda: (
                self.stack.alive("motion_server")
                and self._try_state()
                and all(c.server_is_ready() for c in self.client.clients.values())
            ),
            timeout_s,
            "the motion server's action servers",
        )

    def _try_state(self):
        try:
            return self.client.server_state(timeout_s=2)
        except TimeoutError:
            return None

    def restart_server(self):
        self.stack.restart("motion_server")
        self.wait_for_server()
        self.release()

    def restart_driver(self):
        self.stack.restart("driver")
        self.wait_for_driver()

    # --- the vacuum ---

    def release(self):
        assert self.client.trigger("/release").success, "release failed"

    # --- known-good state ---

    def recover(self):
        """Undo whatever a test did. Raises if the stack cannot be put right."""
        client, stack = self.client, self.stack
        # Faults first: the rest needs a working stack.
        if stack.alive("driver"):
            stack.resume("driver")
        else:
            self.restart_driver()
        client.pause_joint_states(False)
        client.io_mode("ok")
        client.reset_tags()
        if not stack.alive("motion_server"):
            self.restart_server()
        try:
            # The server restarts itself after a trajectory wedged it; wait that out.
            self.wait_for_server()
            wait_until(
                lambda: not client.active_goals(), RECOVERY_S, "the goals to finish"
            )
            state = client.server_state()
        except TimeoutError:
            self.restart_server()
            state = client.server_state()
        assert client.trigger("/resume").success
        if state["world"] != ["table"] or state["attached"]:
            self.restart_server()
        else:
            self.release()
        # Back to a known arm pose, so a test never inherits an awkward one.
        run = client.move_to("home")
        run.wait_done(HOME_TIMEOUT_S)
        if run.outcome != "succeeded":
            self.restart_server()
            self.restart_driver()
            raise RuntimeError(f"could not return home after a test: {run}")
        client.wait_arm_still()

    # --- invariants ---

    def check_invariants(self, stuck_after_s=GOAL_STUCK_S):
        """Raise AssertionError if the server has broken its contract.

        The server process is alive and answering; at most one goal is active; none has
        been active for longer than stuck_after_s.
        """
        assert self.stack.alive("motion_server"), "motion server process died"
        try:
            state = self.client.server_state()
        except TimeoutError:
            raise AssertionError("motion server is not answering /test/state") from None
        # A goal that has just ended can still show as active for a moment while the
        # next one is already accepted, so give the status topic a moment to catch up.
        try:
            wait_until(
                lambda: len(self.client.active_goals()) <= 1,
                ACTIVE_GRACE_S,
                "at most one active goal",
            )
        except TimeoutError:
            active = self.client.active_goals()
            raise AssertionError(
                f"{len(active)} goals active at once: {active}"
            ) from None
        longest = self.client.longest_active_s()
        assert longest < stuck_after_s, f"a goal has been active for {longest:.0f}s"
        return state

    def settle(self, timeout_s=RECOVERY_S):
        """Wait for no active goals and an idle server; return its state."""

        def idle():
            if self.client.active_goals():
                return None
            state = self.client.server_state()
            return state if not state["running"] else None

        self.wait_for_server()
        state = wait_until(idle, timeout_s, "the server to go idle")
        assert not state["busy"] or state["stopped"], (
            f"server is idle but still marked busy, so it will reject goals: {state}"
        )
        return state


@pytest.fixture(scope="session")
def rig(tmp_path_factory):
    env = isolated_env()
    # This process's own rclpy joins the same isolated domain.
    os.environ.update(ROS_DOMAIN_ID=env["ROS_DOMAIN_ID"], ROS_LOCALHOST_ONLY="1")
    log_dir = tmp_path_factory.mktemp("fault_logs")
    stack = Stack(env, log_dir)
    params_file = log_dir / "motion_server_params.yaml"
    server_params(params_file)
    client = None
    try:
        stack.start("driver", driver_command())
        stack.start("fake_tags", [sys.executable, str(HERE / "fake_tags.py")])
        stack.start("fake_io", [sys.executable, str(HERE / "fake_io.py")])
        stack.start("relay", [sys.executable, str(HERE / "joint_state_relay.py")])
        client = Client(int(env["ROS_DOMAIN_ID"]))
        rig = Rig(stack, client)
        rig.wait_for_driver()
        # The motion server waits for the arm; give the relay a moment first.
        wait_until(
            lambda: client.node.count_publishers("/ms/joint_states") > 0,
            60,
            "the joint-state relay",
        )
        stack.start("motion_server", motion_server_command(params_file))
        rig.wait_for_server()
        # The server starts out assuming it holds a cube and lets go once running.
        wait_until(lambda: client.trigger("/release").success, 30, "the vacuum release")
        yield rig
    finally:
        if client is not None:
            client.close()
        stack.stop()
        print(f"fault-test logs: {log_dir}")


@pytest.fixture
def healthy(rig, request):
    """A recovered stack before the test; the server's invariants after it."""
    rig.recover()
    before = rig.client.server_state()
    rig.baseline = before
    started = time.monotonic()
    yield rig
    rig.check_invariants()
    print(f"{request.node.name}: {time.monotonic() - started:.1f}s")
