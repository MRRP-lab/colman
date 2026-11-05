"""Validates a tool call and runs it. The node is the safety boundary.

PickAndPlaceExecutor runs /pick then /place through the callables it is given.
FakeExecutor returns canned results without touching ROS (the node's dry_run mode).
"""

import threading

from colman_llm.tools import DESTINATION_FRAMES, OBJECT_FRAMES


def validate_pick_and_place(args):
    """Return (object_frame, destination_frame, error). Re-checks every value."""
    if not isinstance(args, dict) or set(args) != {"object", "destination"}:
        return None, None, "pick_and_place needs exactly an object and a destination."
    obj, dest = args["object"], args["destination"]
    if not isinstance(obj, str) or obj not in OBJECT_FRAMES:
        return None, None, f"I don't know an object called {obj}."
    if not isinstance(dest, str) or dest not in DESTINATION_FRAMES:
        return None, None, f"I don't know a destination called {dest}."
    return OBJECT_FRAMES[obj], DESTINATION_FRAMES[dest], None


class FakeExecutor:
    def __init__(self, logger):
        self._log = logger

    def run_tool(self, name, args):
        """Return (ok, message) for a tool call."""
        if name != "pick_and_place":
            return False, f"Unknown tool {name}."
        obj_frame, dest_frame, error = validate_pick_and_place(args)
        if error:
            self._log.warning(f"Rejected {name} {args}: {error}")
            return False, error
        self._log.info(
            f"FAKE {name}: {args['object']} ({obj_frame}) -> "
            f"{args['destination']} ({dest_frame})"
        )
        obj, dest = args["object"], args["destination"]
        return True, f"Picked up the {obj} and placed it in the {dest}."


class PickAndPlaceExecutor:
    """pick(frame) and place(frame) each return (ok, message) and may block."""

    def __init__(self, pick, place, logger):
        self._pick = pick
        self._place = place
        self._log = logger
        self._busy = threading.Lock()

    def run_tool(self, name, args):
        """Return (ok, message) for a tool call."""
        if name != "pick_and_place":
            return False, f"Unknown tool {name}."
        obj_frame, dest_frame, error = validate_pick_and_place(args)
        if error:
            self._log.warning(f"Rejected {name} {args}: {error}")
            return False, error
        if not self._busy.acquire(blocking=False):
            return False, "The arm is already busy with another pick and place."
        try:
            obj, dest = args["object"], args["destination"]
            self._log.info(f"{name}: {obj} ({obj_frame}) -> {dest} ({dest_frame})")
            ok, message = self._pick(obj_frame)
            if not ok:
                return False, f"Could not pick up the {obj}. {message}"
            ok, message = self._place(dest_frame)
            if not ok:
                return False, (
                    f"Picked up the {obj} but could not place it in the {dest}. "
                    f"{message}"
                )
            return True, f"Picked up the {obj} and placed it in the {dest}."
        finally:
            self._busy.release()
