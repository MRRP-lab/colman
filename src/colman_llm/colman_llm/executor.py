"""Validates a tool call and runs it. The node is the safety boundary.

FakeExecutor returns canned results without touching ROS. The real executor that
sends /pick and /place goals comes in a later step.
"""

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
