import time

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

BASE_FRAME = "base_link"
TAG_FRAME = "tag_1"
LOOKUP_TIMEOUT_S = 2.0
FRESH_WAIT_S = 0.5
POLL_S = 0.02


class TagLookup(Node):
    def __init__(self):
        super().__init__("tag_lookup")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def lookup(self, tag_frame, base_frame, timeout_s=LOOKUP_TIMEOUT_S):
        """
        Given the tag and the base of the arm,
        returns the latest transform between them, or None.
        """
        try:
            return self.buffer.lookup_transform(
                base_frame,
                tag_frame,
                Time(),
                Duration(seconds=timeout_s),
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

        deadline = time.monotonic() + FRESH_WAIT_S
        while time.monotonic() < deadline:
            time.sleep(POLL_S)
            latest = self.lookup(tag_frame, base_frame, timeout_s=0.0)
            if latest is not None and latest.header.stamp != first.header.stamp:
                return latest.transform
        return None
