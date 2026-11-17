"""Find a tag without moving the arm, and refine an overhead find with the wrist camera.

Two cameras can see a tag. The wrist camera is accurate, but it only sees what the arm
happens to be pointing at. The overhead camera sees the whole table, but its pose for a
39 mm tag from 1.8 m is only good to about 1 cm, too coarse to pick from. So a tag is
looked for in the wrist camera first and then in the overhead camera, and an overhead
find is refined by moving the wrist camera over it ("look over") before it is used.
Nothing here moves the arm except refine_with_wrist.
"""

import time
from collections import namedtuple

WRIST_WAIT_S = 4.0  # how long to wait for the wrist camera to see a tag
LOOK_OVER_WAIT_S = 6.0  # how long to wait for it after moving over an overhead find
LOOK_OVER_HEIGHT_M = 0.30  # how far above the overhead estimate to hold the camera
TOOL_DOWN = (1.0, 0.0, 0.0, 0.0)  # tool pointing straight down

OVERHEAD_PREFIX = "ovh_"  # the overhead detector publishes tag_N as ovh_tag_N

WRIST = "wrist"
OVERHEAD = "overhead"

FOUND = "found"
NOT_VISIBLE = "not_visible"
REFINE_FAILED = "refine_failed"
STOPPED = "stopped"

# source is WRIST or OVERHEAD (None unless found); message explains a failed refine.
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
    stopped=lambda: False,
    on_detect=None,
    wrist_timeout_s=WRIST_WAIT_S,
    clock=time.time,
):
    """Look for tag_frame in the wrist camera, then in the overhead camera.

    The arm does not move. on_detect() runs before the wrist wait, so a caller can
    report progress or ask the camera to refocus. Returns a ScanResult whose source
    says which camera saw the tag; an overhead pose is approximate (see
    refine_with_wrist).
    """
    if on_detect:
        on_detect()
    found = wait_for_tag(tag, tag_frame, wrist_timeout_s, stopped=stopped, clock=clock)
    if found == STOPPED:
        return result(STOPPED)
    if found is not None:
        return result(FOUND, found, WRIST)

    if stopped():
        return result(STOPPED)
    found = tag.get_tag_pose(overhead_frame(tag_frame))
    if found is not None:
        return result(FOUND, found, OVERHEAD)
    return result(NOT_VISIBLE)


def refine_with_wrist(
    arm,
    tag,
    tag_frame,
    estimate,
    params,
    *,
    orientation=TOOL_DOWN,
    height_m=LOOK_OVER_HEIGHT_M,
    timeout_s=LOOK_OVER_WAIT_S,
    stopped=lambda: False,
    on_look=None,
    on_detect=None,
    clock=time.time,
):
    """Move the wrist camera over an overhead estimate and read the tag from there.

    estimate is the overhead pose (a transform). Returns FOUND with the wrist pose, or
    REFINE_FAILED. The overhead estimate is never returned as a result, because it is
    not accurate enough to pick from.
    """
    if stopped():
        return result(STOPPED)
    if on_look:
        on_look()
    t = estimate.translation
    if not arm.go_to_pose(t.x, t.y, t.z + height_m, *orientation, params):
        if stopped():
            return result(STOPPED)
        return result(REFINE_FAILED, message=f"Could not move over {tag_frame}")
    if on_detect:
        on_detect()
    found = wait_for_tag(tag, tag_frame, timeout_s, stopped=stopped, clock=clock)
    if found == STOPPED:
        return result(STOPPED)
    if found is None:
        return result(
            REFINE_FAILED,
            message=f"{tag_frame} not seen by the wrist camera from above it",
        )
    return result(FOUND, found, WRIST)


def locate_tag(
    arm,
    tag,
    tag_frame,
    params,
    *,
    orientation=TOOL_DOWN,
    stopped=lambda: False,
    on_look=None,
    on_detect=None,
    wrist_timeout_s=WRIST_WAIT_S,
    clock=time.time,
):
    """An accurate pose for tag_frame: look for it, and refine an overhead find."""
    scan = look_for_tag(
        tag,
        tag_frame,
        stopped=stopped,
        on_detect=on_detect,
        wrist_timeout_s=wrist_timeout_s,
        clock=clock,
    )
    if scan.status != FOUND or scan.source == WRIST:
        return scan
    return refine_with_wrist(
        arm,
        tag,
        tag_frame,
        scan.tag_pose,
        params,
        orientation=orientation,
        stopped=stopped,
        on_look=on_look,
        on_detect=on_detect,
        clock=clock,
    )
