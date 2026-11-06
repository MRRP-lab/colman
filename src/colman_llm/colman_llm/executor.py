"""Validates a tool call and runs it. The node is the safety boundary.

PickAndPlaceExecutor runs /pick then /place through the callables it is given, and
the /stop and /resume services for the stop_robot and resume_robot tools.
FakeExecutor does the same with callables that always succeed, so the stacking logic
runs without ROS or an arm (the node's dry_run mode).
"""

import threading

from colman_llm.stack_state import StackState
from colman_llm.tools import DESTINATION_FRAMES, MAX_STACK, OBJECT_FRAMES, STACKS


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


def validate_stack_cube(args):
    """Return (color, count, error). Re-checks every value."""
    if not isinstance(args, dict) or set(args) != {"color", "count"}:
        return None, None, "stack_cube needs exactly a color and a count."
    color, count = args["color"], args["count"]
    if not isinstance(color, str) or color not in STACKS:
        return None, None, f"I don't have {color} cubes."
    if isinstance(count, bool) or not isinstance(count, int):
        return None, None, "The count must be a whole number."
    if not 1 <= count <= MAX_STACK:
        return None, None, f"I can stack between 1 and {MAX_STACK} cubes at a time."
    return color, count, None


class PickAndPlaceExecutor:
    """pick(frame) and place(frame) each return (ok, message) and may block."""

    def __init__(self, pick, place, logger, stop=None, resume=None):
        self._pick = pick
        self._place = place
        self._stop = stop
        self._resume = resume
        self._log = logger
        self._busy = threading.Lock()
        self._stacks = StackState()

    def _stop_or_resume(self, name, args):
        """Stop and resume never wait on the busy lock: a stop must always work."""
        if args:
            return False, f"{name} takes no arguments."
        call = self._stop if name == "stop_robot" else self._resume
        if call is None:
            return False, f"{name} is not available."
        ok, message = call()
        self._log.info(f"{name}: ok={ok} {message}")
        if not ok:
            return False, message
        if name == "stop_robot":
            return True, "The arm is stopped."
        return True, "The arm can move again."

    def _stack_cube(self, args):
        color, count, error = validate_stack_cube(args)
        if error:
            self._log.warning(f"Rejected stack_cube {args}: {error}")
            return False, error
        if not self._busy.acquire(blocking=False):
            return False, "The arm is already busy with another stacking job."
        try:
            return self._stack(color, count)
        finally:
            self._busy.release()

    def _stack(self, color, count):
        state = self._stacks
        done = 0
        for _ in range(count):
            move = state.next_move(color)
            if move is None:
                break
            cube, target = move
            name = state.cube_name(color, cube)
            self._log.info(f"stack_cube: {name} ({cube}) onto {target}")
            ok, message = self._pick(cube)
            if not ok:
                return False, self._partial(
                    color, done, count, f"Could not pick up the {name}. {message}"
                )
            ok, message = self._place(target)
            if not ok:
                return False, self._partial(
                    color,
                    done,
                    count,
                    f"Picked up the {name} but could not place it on the stack. "
                    f"{message}",
                )
            state.confirm(color)
            done += 1
        height = state.height(color)
        if done == 0:
            return False, f"The {color} stack is already full."
        message = f"Stacked {done} {color} cube{'s' if done != 1 else ''}. "
        message += f"The {color} stack now has {height} on top of its base."
        if done < count:
            message += " It is full, so I stopped."
        return True, message

    def _partial(self, color, done, count, reason):
        return (
            f"Stacked {done} of {count} {color} cubes. {reason} "
            f"The {color} stack has {self._stacks.height(color)} on top of its base."
        )

    def run_tool(self, name, args):
        """Return (ok, message) for a tool call."""
        if name in ("stop_robot", "resume_robot"):
            return self._stop_or_resume(name, args)
        if name == "stack_cube":
            return self._stack_cube(args)
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


class FakeExecutor(PickAndPlaceExecutor):
    """Canned results: every pick, place, stop and resume succeeds."""

    def __init__(self, logger):
        def pick(frame):
            logger.info(f"FAKE pick {frame}")
            return True, "ok"

        def place(frame):
            logger.info(f"FAKE place {frame}")
            return True, "ok"

        super().__init__(
            pick,
            place,
            logger,
            stop=lambda: (True, "Stopped"),
            resume=lambda: (True, "Resumed"),
        )
