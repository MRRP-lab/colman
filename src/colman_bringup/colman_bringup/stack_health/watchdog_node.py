"""Watches the ROS graph for the nodes, services and actions the stack needs.

Publishes `diagnostic_msgs/DiagnosticArray` on `/stack_health` once a second, one
status per expected component plus an overall `stack` entry (OK, WARN while
starting, ERROR when something is missing). Logs loudly when something
disappears and can speak about it on `llm_response`.

Motion server safety, because it can die with the vacuum on and comes back
accepting goals immediately: when it disappears the watchdog switches the
vacuum off through the UR IO service, and when it returns it calls `/stop` so
the arm stays put until someone says resume.
"""

import time

import rclpy
from colman_bringup.stack_health.health import (
    ACTION,
    NODE,
    SERVICE,
    Component,
    HealthMonitor,
    Reaction,
    Snapshot,
    Status,
    plan_reactions,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

DEFAULT_NODES = [
    "motion_server",
    "llm_node",
    "kokoro_tts",
    "speech_to_text",
    "driver_supervisor",
    "camera_publisher",
]
DEFAULT_SERVICES = ["/stop", "/resume"]
DEFAULT_ACTIONS = ["/pick", "/place", "/move_to"]
VACUUM_PIN = 0
LEVEL = {
    Status.ALIVE: DiagnosticStatus.OK,
    Status.STARTING: DiagnosticStatus.WARN,
    Status.MISSING: DiagnosticStatus.ERROR,
}


def graph_snapshot(node):
    """Names of the live nodes (as `/name`) and services in the graph."""
    names = {
        ns.rstrip("/") + "/" + name for name, ns in node.get_node_names_and_namespaces()
    }
    services = {name for name, _ in node.get_service_names_and_types()}
    return Snapshot(nodes=names, services=services)


class StackHealth(Node):
    def __init__(self):
        super().__init__("stack_health")
        self.declare_parameter("nodes", DEFAULT_NODES)
        self.declare_parameter("services", DEFAULT_SERVICES)
        self.declare_parameter("actions", DEFAULT_ACTIONS)
        self.declare_parameter("grace", 3.0)
        self.declare_parameter("startup_timeout", 60.0)
        self.declare_parameter("period", 1.0)
        self.declare_parameter("speak", True)
        self.declare_parameter("speak_cooldown", 20.0)
        self.declare_parameter("motion_node", "motion_server")
        self.declare_parameter("release_vacuum_on_motion_loss", True)
        self.declare_parameter("stop_on_motion_return", True)
        get = lambda name: self.get_parameter(name).value  # noqa: E731

        expected = (
            [Component(NODE, "/" + n.lstrip("/")) for n in get("nodes")]
            + [Component(SERVICE, s) for s in get("services")]
            + [Component(ACTION, a) for a in get("actions")]
        )
        self._monitor = HealthMonitor(expected, get("grace"), get("startup_timeout"))
        self._motion_node = Component(NODE, "/" + get("motion_node").lstrip("/"))
        self._stop_service = Component(SERVICE, "/stop")
        self._release_vacuum = get("release_vacuum_on_motion_loss")
        self._stop_on_return = get("stop_on_motion_return")
        self._speak = get("speak")
        self._speak_cooldown = get("speak_cooldown")
        self._last_spoken = None

        self._pub = self.create_publisher(DiagnosticArray, "/stack_health", 10)
        self._say = self.create_publisher(String, "llm_response", 10)
        self._stop_client = self.create_client(Trigger, "/stop")
        self._io_client = None  # created on first use, see _release
        self.create_timer(get("period"), self._tick)
        self.get_logger().info(
            f"Watching {len(expected)} components; publishing /stack_health"
        )

    def _tick(self):
        now = time.monotonic()
        changes = self._monitor.update(graph_snapshot(self), now)
        log = self.get_logger()
        for ch in changes:
            msg = f"{ch.event.name}: {ch.component.label}"
            if ch.event.name == "RECOVERED":
                log.warn(msg)
            else:
                log.error(msg)
        for reaction, text in plan_reactions(
            changes,
            self._motion_node,
            self._stop_service,
            self._release_vacuum,
            self._stop_on_return,
        ):
            self._react(reaction, text, now)
        self._publish()

    def _react(self, reaction, text, now):
        if reaction == Reaction.SAY:
            if not self._speak:
                return
            last = self._last_spoken
            if last is not None and now - last < self._speak_cooldown:
                return
            self._last_spoken = now
            self._say.publish(String(data=text))
        elif reaction == Reaction.STOP_MOTION:
            self.get_logger().error(
                "Motion server is back: calling /stop; `pixi run resume` to continue"
            )
            if self._stop_client.service_is_ready():
                self._stop_client.call_async(Trigger.Request())
            else:
                self.get_logger().error("/stop is not ready, arm is NOT stopped")
        elif reaction == Reaction.RELEASE_VACUUM:
            self._release()

    def _release(self):
        from ur_msgs.srv import SetIO

        if self._io_client is None:
            self._io_client = self.create_client(
                SetIO, "/io_and_status_controller/set_io"
            )
        if not self._io_client.service_is_ready():
            self.get_logger().error("Cannot release vacuum: set_io is not available")
            return
        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin = VACUUM_PIN
        req.state = 0.0
        self._io_client.call_async(req)
        self.get_logger().warn("Motion server gone: releasing the vacuum")

    def _publish(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for c in self._monitor.expected:
            status = self._monitor.status[c]
            msg.status.append(
                DiagnosticStatus(
                    level=LEVEL[status],
                    name=f"colman/{c.label}",
                    message=status.name.lower(),
                    hardware_id="colman_stack",
                    values=[KeyValue(key="kind", value=c.kind)],
                )
            )
        missing, starting = self._monitor.summary()
        if missing:
            level, text = DiagnosticStatus.ERROR, "missing: " + ", ".join(missing)
        elif starting:
            level, text = DiagnosticStatus.WARN, "starting"
        else:
            level, text = DiagnosticStatus.OK, "all alive"
        msg.status.insert(
            0,
            DiagnosticStatus(
                level=level,
                name="colman/stack",
                message=text,
                hardware_id="colman_stack",
            ),
        )
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StackHealth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
