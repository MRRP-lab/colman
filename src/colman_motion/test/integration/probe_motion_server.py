"""The real motion server plus a /test/state service for the tests to inspect it.

Nothing about the server's behaviour changes. The service reports what a test cannot
see from outside: the planning scene (world objects and attached objects), the arm's
busy/stopped flags and the vacuum's idea of whether it holds a cube.
"""

import json
import os
import threading

import colman_motion.motion_server as motion_server
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger


class ProbedMotionServer(motion_server.MotionServer):
    def __init__(self, arm, tag, vacuum):
        super().__init__(arm, tag, vacuum)
        self.create_service(
            Trigger,
            "/test/state",
            self.state_cb,
            callback_group=ReentrantCallbackGroup(),
        )

    def state_cb(self, request, response):
        psm = self.arm.moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            msg = scene.planning_scene_message
        response.success = True
        response.message = json.dumps(
            {
                "pid": os.getpid(),
                "threads": threading.active_count(),
                "world": sorted(o.id for o in msg.world.collision_objects),
                "attached": sorted(
                    a.object.id for a in msg.robot_state.attached_collision_objects
                ),
                "busy": self.arm.busy,
                "running": self.arm.running,
                "stopped": self.arm.stopped,
                "stop_event": self.arm.stop_event.is_set(),
                "holding": self.vacuum.holding,
            }
        )
        return response


if __name__ == "__main__":
    motion_server.MotionServer = ProbedMotionServer
    motion_server.main()
