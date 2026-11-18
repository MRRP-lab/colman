"""Stand-in for the UR driver's set_io service, with faults on demand.

The motion server is started with /io_and_status_controller/set_io remapped to
/test_io/set_io, so a test can make the vacuum fail or hang without touching the
driver. Control and read-back go through this node's parameters:

  mode   "ok" | "fail" (success=False) | "hang" (no reply until mode changes)
  pin0   the vacuum output as last set
  calls  number of set_io requests received
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ur_msgs.srv import SetIO

HANG_POLL_S = 0.05
HANG_MAX_S = 120.0


class FakeIO(Node):
    def __init__(self):
        super().__init__("fake_io")
        self.declare_parameter("mode", "ok")
        self.declare_parameter("pin0", False)
        self.declare_parameter("calls", 0)
        self.calls = 0
        self.create_service(
            SetIO,
            "/test_io/set_io",
            self.set_io,
            callback_group=ReentrantCallbackGroup(),
        )

    def mode(self):
        return self.get_parameter("mode").value

    def set_io(self, request, response):
        self.calls += 1
        self.set_parameters([Parameter("calls", value=self.calls)])
        deadline = time.monotonic() + HANG_MAX_S
        while self.mode() == "hang" and time.monotonic() < deadline:
            time.sleep(HANG_POLL_S)
        if self.mode() == "fail":
            response.success = False
            return response
        if request.pin == 0:
            self.set_parameters([Parameter("pin0", value=bool(request.state))])
        response.success = True
        return response


def main():
    rclpy.init()
    node = FakeIO()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
