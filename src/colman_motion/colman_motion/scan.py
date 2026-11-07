"""Look for a tag from each scan pose in turn.

One scan pose cannot see the whole table, so a tag that is not visible from the first
pose may be visible from the next. The pose that last saw a tag is tried first, so
repeated picks from one side do not waste a look at the other.
"""

import time
from collections import namedtuple

from colman_motion.poses import SCAN_POSES

FIRST_LOOK_TIMEOUT_S = 4.0  # how long to wait at a pose before trying the next one
LAST_LOOK_TIMEOUT_S = 10.0  # the last pose gets the full wait

FOUND = "found"
NOT_VISIBLE = "not_visible"
LOOK_FAILED = "look_failed"
STOPPED = "stopped"

ScanResult = namedtuple("ScanResult", ["status", "tag_pose", "pose"])


def scan_order(prefer=None, poses=SCAN_POSES):
    """The scan poses with the preferred one (if any) first."""
    order = list(poses)
    if prefer in order:
        order.remove(prefer)
        order.insert(0, prefer)
    return order


def look_for_tag(
    arm,
    tag,
    tag_frame,
    params,
    *,
    stopped=lambda: False,
    prefer=None,
    on_look=None,
    on_detect=None,
    first_timeout_s=FIRST_LOOK_TIMEOUT_S,
    last_timeout_s=LAST_LOOK_TIMEOUT_S,
    poses=SCAN_POSES,
    clock=time.time,
):
    """Move to each scan pose until tag_frame is seen. Return a ScanResult.

    on_look(pose) runs before each move and on_detect(pose) before each wait, so a
    caller can report progress or ask the camera to refocus.
    """
    order = scan_order(prefer, poses)
    for index, pose in enumerate(order):
        if stopped():
            return ScanResult(STOPPED, None, pose)
        if on_look:
            on_look(pose)
        if not arm.go_to_configured_pose(pose, params):
            return ScanResult(LOOK_FAILED, None, pose)
        if on_detect:
            on_detect(pose)
        timeout_s = last_timeout_s if index == len(order) - 1 else first_timeout_s
        deadline = clock() + timeout_s
        while clock() < deadline:
            if stopped():
                return ScanResult(STOPPED, None, pose)
            tag_pose = tag.get_tag_pose(tag_frame)
            if tag_pose is not None:
                return ScanResult(FOUND, tag_pose, pose)
    return ScanResult(NOT_VISIBLE, None, order[-1])
