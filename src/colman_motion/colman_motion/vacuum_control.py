import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger
from ur_msgs.srv import SetIO

VACUUM_PIN = 0


class VacuumControl(Node):
    def __init__(self):
        super().__init__("vacuum_control")
        self.holding = True
        self.vacuum = self.create_client(
            SetIO,
            "/io_and_status_controller/set_io",
            callback_group=ReentrantCallbackGroup(),
        )
        while not self.vacuum.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for /io_and_status_controller/set_io")
        self.create_service(Trigger, "/grasp", self.grasp_cb)
        self.create_service(Trigger, "/release", self.release_cb)

    def grasp_cb(self, request, response):
        response.success = self.grasp()
        response.message = "Vacuum on"
        return response

    def release_cb(self, request, response):
        response.success = self.release()
        response.message = "Vacuum off"
        return response

    def grasp(self):
        self.holding = True
        return self.set_digital_out(VACUUM_PIN, True)

    def release(self):
        released = self.set_digital_out(VACUUM_PIN, False)
        if released:
            self.holding = False
        return released

    def set_digital_out(self, pin, state):
        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin = pin
        req.state = float(state)
        future = self.vacuum.call_async(req)
        deadline = time.monotonic() + 2.0
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error("set_io timed out")
                return False
            time.sleep(0.001)
        result = future.result()
        if result is None:
            self.get_logger().error("set_io failed")
            return False
        return result.success
