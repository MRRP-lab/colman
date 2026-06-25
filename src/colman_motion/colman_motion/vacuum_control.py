import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from ur_msgs.srv import SetIO

VACUUM_PIN = 0


class VacuumControl(Node):
    def __init__(self):
        super().__init__("vacuum_control")
        self.vacuum = self.create_client(
            SetIO,
            "/io_and_status_controller/set_io",
            callback_group=ReentrantCallbackGroup(),
        )
        self.vacuum.wait_for_service()

    def grasp(self):
        return self.set_digital_out(VACUUM_PIN, True)

    def release(self):
        return self.set_digital_out(VACUUM_PIN, False)

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
