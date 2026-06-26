import time

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

BASE_FRAME = "base_link"
TAG_FRAME = "tag_1"
LOOKUP_TIMEOUT_S = 2.0
TOLERANCE_M = 0.003
WAIT_S = 1.0


def within_tolerance(first, second, tol):
    """
    Checks that the tag hasn't moved past the specified threshold.
    """
    dx = abs(second.x - first.x)
    dy = abs(second.y - first.y)
    dz = abs(second.z - first.z)
    return max(dx, dy, dz) < tol


class TagLookup(Node):
    def __init__(self):
        super().__init__("tag_lookup")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def lookup(self, tag_frame, base_frame):
        """
        Given the tag and the base of the arm,
        returns the latest transform between them, or None.
        """
        try:
            return self.buffer.lookup_transform(
                base_frame,
                tag_frame,
                Time(),
                Duration(seconds=LOOKUP_TIMEOUT_S),
            )
        except TransformException:
            return None

    def get_tag_pose(self, tag_frame=TAG_FRAME, base_frame=BASE_FRAME):
        """
        Returns the tag's transform once it has settled,
        or None while it is still moving or unseen.
        """
        first = self.lookup(tag_frame, base_frame)

        if first is None:
            return None

        time.sleep(WAIT_S)

        second = self.lookup(tag_frame, base_frame)
        if second is None:
            return None

        if first.header.stamp == second.header.stamp:
            return None

        first_pos = first.transform.translation
        second_pos = second.transform.translation

        if not within_tolerance(first_pos, second_pos, TOLERANCE_M):
            return None

        return second.transform
