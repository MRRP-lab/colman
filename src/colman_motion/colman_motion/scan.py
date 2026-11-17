"""Find a tag in the wrist camera or the overhead camera, without scanning poses.

The wrist camera gets one look, at once, from wherever the arm is. If it does not see
the tag, the overhead camera is used; its pose is accurate enough to pick and place
from. If neither sees the tag the arm may be blocking the overhead view, so the arm
moves to HOME_POSE and the overhead camera is tried again.
"""

import time
from collections import namedtuple

from colman_motion.poses import HOME_POSE

WRIST_WAIT_S = 0.0  # the wrist camera only gets a look at once, it never waits
OVERHEAD_WAIT_S = 4.0  # how long to wait for the overhead camera after clearing it

OVERHEAD_PREFIX = "ovh_"  # the overhead detector publishes tag_N as ovh_tag_N

WRIST = "wrist"
OVERHEAD = "overhead"

FOUND = "found"
NOT_VISIBLE = "not_visible"
CLEAR_FAILED = "clear_failed"
STOPPED = "stopped"

# source is WRIST or OVERHEAD (None unless found); message explains a failed move.
ScanResult = namedtuple("ScanResult", ["status", "tag_pose", "source", "message"])


def result(status, tag_pose=None, source=None, message=""):
    return ScanResult(status, tag_pose, source, message)


def overhead_frame(tag_frame):
    return OVERHEAD_PREFIX + tag_frame


def wait_for_tag(tag, tag_frame, timeout_s, *, stopped=lambda: False, clock=time.time):
    """Wait up to timeout_s for tag_frame. Return its pose, None, or STOPPED.

    The tag is always looked for at least once, so a timeout of 0 is a single look.
    """
    deadline = clock() + timeout_s
    while True:
        if stopped():
            return STOPPED
        tag_pose = tag.get_tag_pose(tag_frame)
        if tag_pose is not None:
            return tag_pose
        if clock() >= deadline:
            return None


def look_for_tag(
    tag,
    tag_frame,
    *,
    arm=None,
    params=None,
    stopped=lambda: False,
    on_look=None,
    on_detect=None,
    wrist_timeout_s=WRIST_WAIT_S,
    overhead_timeout_s=OVERHEAD_WAIT_S,
    clock=time.time,
):
    """Look for tag_frame in the wrist camera, then in the overhead camera.

    The wrist camera gets a look at once, without moving the arm. If it does not see
    the tag, the overhead camera is tried. If that does not see it either and an arm
    is given, the arm may be in the way, so it moves to HOME_POSE and the overhead
    camera is tried again. Without an arm nothing moves.

    on_look() runs before that move and on_detect() before the wrist look, so a caller
    can report progress or ask the camera to refocus. Returns a ScanResult whose source
    says which camera saw the tag.
    """
    if on_detect:
        on_detect()
    found = wait_for_tag(tag, tag_frame, wrist_timeout_s, stopped=stopped, clock=clock)
    if found == STOPPED:
        return result(STOPPED)
    if found is not None:
        return result(FOUND, found, WRIST)

    overhead = overhead_frame(tag_frame)
    found = wait_for_tag(tag, overhead, 0.0, stopped=stopped, clock=clock)
    if found == STOPPED:
        return result(STOPPED)
    if found is not None:
        return result(FOUND, found, OVERHEAD)
    if arm is None:
        return result(NOT_VISIBLE)

    if on_look:
        on_look()
    if not arm.go_to_joint_pose(HOME_POSE, params):
        if stopped():
            return result(STOPPED)
        return result(CLEAR_FAILED, message="Could not move the arm out of the view")
    found = wait_for_tag(
        tag, overhead, overhead_timeout_s, stopped=stopped, clock=clock
    )
    if found == STOPPED:
        return result(STOPPED)
    if found is not None:
        return result(FOUND, found, OVERHEAD)
    return result(NOT_VISIBLE)


def locate_tag(
    arm,
    tag,
    tag_frame,
    params,
    *,
    stopped=lambda: False,
    on_look=None,
    on_detect=None,
    wrist_timeout_s=WRIST_WAIT_S,
    clock=time.time,
):
    """The pose of tag_frame from whichever camera sees it. See look_for_tag."""
    return look_for_tag(
        tag,
        tag_frame,
        arm=arm,
        params=params,
        stopped=stopped,
        on_look=on_look,
        on_detect=on_detect,
        wrist_timeout_s=wrist_timeout_s,
        clock=clock,
    )
