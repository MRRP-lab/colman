"""ROS node that keeps a UR arm connected and ready, recovering in place.

Watches the arm and the driver and carries out the actions chosen by
`SupervisorMachine`. It also owns the `ur_control.launch.py` process: it starts
it once the arm is reachable and restarts it if it never comes up.
"""

import os
import signal
import socket
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from ur_dashboard_msgs.action import SetMode
from ur_dashboard_msgs.msg import RobotMode, SafetyMode

from colman_bringup.driver_supervisor.driver_process import DriverProcess
from colman_bringup.driver_supervisor.state_machine import (
    ROBOT_MODE_RUNNING,
    Action,
    Config,
    Observation,
    State,
    SupervisorMachine,
)

DASHBOARD_PORT = 29999
PROBE_PERIOD = 1.0
PROBE_TIMEOUT = 1.0
IO_CONTROLLER = "io_and_status_controller"


class DriverSupervisor(Node):
    def __init__(self):
        super().__init__("driver_supervisor")
        self.declare_parameter("robot_ip", "192.168.56.101")
        self.declare_parameter(
            "motion_controller", "scaled_joint_trajectory_controller"
        )
        self.declare_parameter("auto_recover_protective_stop", True)
        self.declare_parameter("link_grace", 5.0)
        self.declare_parameter("driver_grace", 5.0)
        self.declare_parameter("state_grace", 1.0)
        self.declare_parameter("retry_interval", 5.0)
        self.declare_parameter("driver_timeout", 30.0)
        # How long the controller manager may stay silent before it counts as gone.
        self.declare_parameter("status_stale_after", 2.0)
        self.declare_parameter("tick_rate", 4.0)
        self.declare_parameter("ur_type", "ur3e")
        self.declare_parameter("headless_mode", True)
        self.declare_parameter("use_camera", True)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self._ip = get("robot_ip")
        self._motion_controller = get("motion_controller")
        self._stale_after = get("status_stale_after")
        self._machine = SupervisorMachine(
            Config(
                auto_recover_protective_stop=get("auto_recover_protective_stop"),
                link_grace=get("link_grace"),
                driver_grace=get("driver_grace"),
                state_grace=get("state_grace"),
                retry_interval=get("retry_interval"),
                driver_timeout=get("driver_timeout"),
            )
        )

        # The driver publishes these only on change, latched, so match its QoS
        # to receive the current value on connect.
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._robot_mode = None
        self._safety_mode = None
        self._program_running = None
        self.create_subscription(
            RobotMode,
            f"/{IO_CONTROLLER}/robot_mode",
            lambda m: self._record("robot_mode", m.mode),
            status_qos,
        )
        self.create_subscription(
            SafetyMode,
            f"/{IO_CONTROLLER}/safety_mode",
            lambda m: self._record("safety_mode", m.mode),
            status_qos,
        )
        self.create_subscription(
            Bool,
            f"/{IO_CONTROLLER}/robot_program_running",
            lambda m: self._record("program_running", m.data),
            status_qos,
        )

        self._list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self._resend_client = self.create_client(
            Trigger, f"/{IO_CONTROLLER}/resend_robot_program"
        )
        self._mode_client = ActionClient(
            self, SetMode, "/ur_robot_state_helper/set_mode"
        )

        self._cm_ok_at = None
        self._io_active = False
        self._motion_active = None
        self._list_pending = False
        self._in_flight = set()

        self._link_up = False
        self._stop = threading.Event()
        threading.Thread(target=self._probe_link, daemon=True).start()

        self._driver = DriverProcess(self._driver_command(), self.get_logger())

        self._state_pub = self.create_publisher(
            String,
            "/driver_supervisor/state",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._published = None
        self.create_timer(1.0 / get("tick_rate"), self._tick)
        self.get_logger().info(f"Supervising arm at {self._ip}")

    def _driver_command(self):
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        rsp = os.path.join(
            get_package_share_directory("colman_bringup"), "launch", "rsp.launch.py"
        )
        return [
            "ros2",
            "launch",
            "ur_robot_driver",
            "ur_control.launch.py",
            f"ur_type:={get('ur_type')}",
            f"robot_ip:={self._ip}",
            f"headless_mode:={str(get('headless_mode')).lower()}",
            f"use_camera:={str(get('use_camera')).lower()}",
            f"description_launchfile:={rsp}",
        ]

    def destroy_node(self):
        self._stop.set()
        self._driver.stop()
        super().destroy_node()

    def _record(self, key, value):
        setattr(self, f"_{key}", value)

    def _latest(self, key):
        return getattr(self, f"_{key}")

    def _probe_link(self):
        """TCP connect to the dashboard server: needs no ROS node to be alive."""
        while not self._stop.is_set():
            try:
                with socket.create_connection(
                    (self._ip, DASHBOARD_PORT), PROBE_TIMEOUT
                ):
                    self._link_up = True
            except OSError:
                self._link_up = False
            self._stop.wait(PROBE_PERIOD)

    def _poll_controllers(self):
        if self._list_pending or not self._list_client.service_is_ready():
            return
        self._list_pending = True
        future = self._list_client.call_async(ListControllers.Request())

        def done(f):
            self._list_pending = False
            if f.exception() is not None or f.result() is None:
                return
            states = {c.name: c.state for c in f.result().controller}
            self._cm_ok_at = time.monotonic()
            self._io_active = states.get(IO_CONTROLLER) == "active"
            self._motion_active = states.get(self._motion_controller) == "active"

        future.add_done_callback(done)

    def _observe(self):
        now = time.monotonic()
        cm_ok = (
            self._cm_ok_at is not None and now - self._cm_ok_at < self._stale_after * 2
        )
        return Observation(
            now=now,
            link_up=self._link_up,
            driver_up=cm_ok and self._io_active,
            robot_mode=self._latest("robot_mode"),
            safety_mode=self._latest("safety_mode"),
            program_running=self._latest("program_running"),
            controller_active=self._motion_active if cm_ok else None,
        )

    def _tick(self):
        self._poll_controllers()
        before = self._machine.state
        self._ensure_driver()
        for action in self._machine.update(self._observe()):
            self._perform(action)
        if self._machine.state != before:
            self.get_logger().info(
                f"{before.name} -> {self._machine.state.name}: {self._machine.reason}"
            )
        self._publish_state()

    def _ensure_driver(self):
        """Keep a driver running whenever the arm is reachable.

        Starting it with no arm attached just makes it fail and give up, so
        wait for the link first. A crashed driver is restarted here too.
        """
        if self._driver.running:
            return
        if self._machine.state == State.WAITING_FOR_LINK:
            return
        self._driver.start()

    def _publish_state(self):
        text = self._machine.state.name
        if text != self._published:
            self._published = text
            self._state_pub.publish(String(data=text))

    def _perform(self, action):
        if action in self._in_flight:
            return
        log = self.get_logger()
        if action == Action.SET_MODE_RUNNING:
            if not self._mode_client.server_is_ready():
                log.warn("set_mode action server not available yet")
                return
            goal = SetMode.Goal(
                target_robot_mode=ROBOT_MODE_RUNNING,
                stop_program=True,
                play_program=False,
            )
            log.info("Requesting robot mode RUNNING")
            self._begin(action)
            self._mode_client.send_goal_async(goal).add_done_callback(
                lambda f: self._goal_accepted(action, f)
            )
        elif action == Action.RESEND_PROGRAM:
            if not self._resend_client.service_is_ready():
                log.warn("resend_robot_program service not available yet")
                return
            log.info("Resending robot program")
            self._call(action, self._resend_client, Trigger.Request())
        elif action == Action.ACTIVATE_CONTROLLER:
            if not self._switch_client.service_is_ready():
                return
            request = SwitchController.Request(
                activate_controllers=[self._motion_controller],
                strictness=SwitchController.Request.BEST_EFFORT,
            )
            log.info(f"Activating {self._motion_controller}")
            self._call(action, self._switch_client, request)
        elif action == Action.RESTART_DRIVER:
            log.warn("Driver is not coming up, restarting it")
            self._driver.restart()

    def _begin(self, action):
        self._in_flight.add(action)

    def _end(self, action):
        self._in_flight.discard(action)

    def _call(self, action, client, request):
        self._begin(action)
        client.call_async(request).add_done_callback(lambda f: self._end(action))

    def _goal_accepted(self, action, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("set_mode goal rejected")
            self._end(action)
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._mode_done(action, f)
        )

    def _mode_done(self, action, future):
        result = future.result().result
        if not result.success:
            self.get_logger().warn(f"set_mode failed: {result.message}")
        self._end(action)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = DriverSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
