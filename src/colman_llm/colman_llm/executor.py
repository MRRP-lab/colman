"""Validates a tool call and runs it. The node is the safety boundary.

PickAndPlaceExecutor runs /pick then /place through the callables it is given, the
/stop and /resume services for stop_robot and resume_robot, /release for release_cube
and /move_to for go_to_pose. report_state only reads the node's own state.
FakeExecutor does the same with callables that always succeed, so the stacking logic
runs without ROS or an arm (the node's dry_run mode).
"""

import threading
import time

from colman_llm.tools import COLORS, CUBE_NAMES, CUBES_PER_COLOR, MIN_STACK, POSES
from colman_llm.world_view import NAME_OF

HELD_HINT = "The arm may still be holding it. Say release to let go of it."
NO_VIEW = "I can't see the table right now, so I won't move."
CONFIRM_TRIES = 20  # how many times to look for a placed cube before giving up
CONFIRM_WAIT_S = 0.4
MOVING_TOOLS = ("stack_cube", "pick_and_place", "go_to_pose")
SETTLE_TRIES = 12  # how long to wait for a cube that is not in view yet


def validate_pick_and_place(args):
    """Return (object_frame, destination_frame, error). Re-checks every value."""
    if not isinstance(args, dict) or set(args) != {"object", "destination"}:
        return None, None, "pick_and_place needs exactly an object and a destination."
    obj, dest = args["object"], args["destination"]
    if not isinstance(obj, str) or obj not in CUBE_NAMES:
        return None, None, f"I don't know an object called {obj}."
    if not isinstance(dest, str) or dest not in CUBE_NAMES:
        return None, None, f"I don't know a destination called {dest}."
    return CUBE_NAMES[obj], CUBE_NAMES[dest], None


def validate_go_to_pose(args):
    """Return (pose_name, error). Re-checks every value."""
    if not isinstance(args, dict) or set(args) != {"pose"}:
        return None, "go_to_pose needs exactly a pose."
    pose = args["pose"]
    if not isinstance(pose, str) or pose not in POSES:
        return None, f"I don't have a pose called {pose}."
    return POSES[pose], None


def validate_stack_cube(args):
    """Return (color, count, error). Re-checks every value."""
    if not isinstance(args, dict) or set(args) != {"color", "count"}:
        return None, None, "stack_cube needs exactly a color and a count."
    color, count = args["color"], args["count"]
    if not isinstance(color, str) or color not in COLORS:
        return None, None, f"I don't have {color} cubes."
    if isinstance(count, bool) or not isinstance(count, int):
        return None, None, "The count must be a whole number."
    if not MIN_STACK <= count <= CUBES_PER_COLOR:
        return (
            None,
            None,
            (
                f"A stack needs between {MIN_STACK} and {CUBES_PER_COLOR} cubes of one "
                "colour."
            ),
        )
    return color, count, None


class PickAndPlaceExecutor:
    """pick(frame, place_frame) and place(frame) return (ok, message) and may block.

    pick is told which cube the picked one will go on, so the motion server can refuse
    before it lifts anything if it cannot place there.

    world() returns the latest WorldView (what the overhead camera sees), or None if
    there is none. Which cube is on which is read from it every time, never remembered.
    """

    def __init__(
        self,
        pick,
        place,
        logger,
        world,
        stop=None,
        resume=None,
        release=None,
        move_to=None,
        sleep=time.sleep,
    ):
        self._world = world
        self._sleep = sleep
        self._pick = pick
        self._place = place
        self._stop = stop
        self._resume = resume
        self._release = release
        self._move_to = move_to
        self._log = logger
        self._busy = threading.Lock()
        self._held = None  # frame of the cube the arm believes it is holding
        self._stopped = False

    @property
    def stopped(self):
        return self._stopped

    def set_stopped(self, stopped):
        """The node calls this when a spoken stop reaches the arm outside run_tool."""
        self._stopped = stopped

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
        self._stopped = name == "stop_robot"
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
            if self._held is not None:
                return False, self._holding_message()
            return self._stack(color, count)
        finally:
            self._busy.release()

    def _look(self):
        return self._world()

    def _look_until(self, verdict):
        """Look until verdict(view) has no "can't see" complaint, for a few seconds.

        The cameras need a moment after the arm moves, so a cube that is not seen yet
        is usually seen a second later. Returns (view, complaint).
        """
        for _ in range(SETTLE_TRIES):
            view = self._look()
            complaint = verdict(view) if view is not None else None
            if view is None or not complaint or "can't see" not in complaint:
                return view, complaint
            self._sleep(CONFIRM_WAIT_S)
        view = self._look()
        return view, (verdict(view) if view is not None else None)

    def _confirm(self, cube, target):
        """Wait until the camera sees the cube on top of the target's column."""
        for _ in range(CONFIRM_TRIES):
            view = self._look()
            if view is not None and view.on_top_of(cube, target):
                return view
            self._sleep(CONFIRM_WAIT_S)
        return None

    def _stack(self, color, count):
        """Grow the colour's tallest stack to count cubes, one cube at a time."""
        done = 0
        why = ""
        for _ in range(count):
            view, _ = self._look_until(lambda v: v.next_move(color, count)[2])
            if view is None:
                return False, self._partial(color, done, count, NO_VIEW)
            cube, target, reason = view.next_move(color, count)
            if cube is None:
                if done == 0:
                    already = view.height(color) >= count
                    return already, reason
                why = reason
                break
            name = view.name_of(cube)
            self._log.info(f"stack_cube: {name} ({cube}) onto {target}")
            ok, message = self._pick(cube, target)
            if not ok:
                return False, self._partial(
                    color, done, count, f"Could not pick up the {name}. {message}"
                )
            self._held = cube
            # Someone may have changed the stack while the arm was picking.
            view = self._look()
            new_target, reason = (
                (None, NO_VIEW) if view is None else view.target_of(color)
            )
            if new_target is None:
                return False, self._partial(
                    color, done, count, f"Picked up the {name}. {reason} {HELD_HINT}"
                )
            if new_target != target:
                self._log.info(f"stack_cube: top changed, {target} -> {new_target}")
                target = new_target
            ok, message = self._place(target)
            if not ok:
                return False, self._partial(
                    color,
                    done,
                    count,
                    f"Picked up the {name} but could not place it on the stack. "
                    f"{message} {HELD_HINT}",
                )
            self._held = None
            if self._confirm(cube, target) is None:
                return False, self._partial(
                    color,
                    done,
                    count,
                    f"I placed the {name} but the camera does not see it on the "
                    f"{view.name_of(target)}.",
                )
            done += 1
        view = self._look()
        height = view.height(color) if view is not None else 0
        message = (
            f"Added {done} {color} cube{'s' if done != 1 else ''}. "
            f"The tallest {color} stack now has {height}."
        )
        if height < count:
            message += f" {why} I stopped."
        return True, message

    def _partial(self, color, done, count, reason):
        view = self._look()
        height = view.height(color) if view is not None else 0
        return (
            f"Added {done} {color} cubes toward a stack of {count}. {reason} "
            f"The tallest {color} stack has {height}."
        )

    def _pick_and_place(self, args):
        obj_frame, dest_frame, error = validate_pick_and_place(args)
        if error:
            self._log.warning(f"Rejected pick_and_place {args}: {error}")
            return False, error
        if not self._busy.acquire(blocking=False):
            return False, "The arm is already busy with another job."
        try:
            if self._held is not None:
                return False, self._holding_message()
            view, reason = self._look_until(
                lambda v: v.check_move(obj_frame, dest_frame)
            )
            if view is None:
                return False, NO_VIEW
            if reason:
                self._log.warning(f"Rejected pick_and_place {args}: {reason}")
                return False, reason
            obj, dest = args["object"], args["destination"]
            self._log.info(
                f"pick_and_place: {obj} ({obj_frame}) -> {dest} ({dest_frame})"
            )
            ok, message = self._pick(obj_frame, dest_frame)
            if not ok:
                return False, f"Could not pick up the {obj}. {message}"
            self._held = obj_frame
            # Someone may have changed the destination while the arm was picking.
            view = self._look()
            if view is None or view.top_of(dest_frame) != dest_frame:
                return False, (
                    f"Picked up the {obj} but the {dest} is no longer free to place "
                    f"on. {HELD_HINT}"
                )
            ok, message = self._place(dest_frame)
            if not ok:
                return False, (
                    f"Picked up the {obj} but could not place it on the {dest}. "
                    f"{message} {HELD_HINT}"
                )
            self._held = None
            if self._confirm(obj_frame, dest_frame) is None:
                return False, (
                    f"I placed the {obj} but the camera does not see it on the {dest}."
                )
            return True, f"Picked up the {obj} and placed it on the {dest}."
        finally:
            self._busy.release()

    def _holding_message(self):
        name = NAME_OF[self._held]
        return f"The arm is still holding the {name}. Release it first. {HELD_HINT}"

    def _release_cube(self, args):
        if args:
            return False, "release_cube takes no arguments."
        if self._release is None:
            return False, "release_cube is not available."
        if not self._busy.acquire(blocking=False):
            return False, "The arm is busy moving, so I won't let go now."
        try:
            ok, message = self._release()
            self._log.info(f"release_cube: ok={ok} {message}")
            if not ok:
                return False, message
            if self._held is None:
                return True, "The vacuum is off. I wasn't holding a cube."
            name = NAME_OF[self._held]
            self._held = None
            return True, f"Released the {name}. It is loose on the table."
        finally:
            self._busy.release()

    def _go_to_pose(self, args):
        pose, error = validate_go_to_pose(args)
        if error:
            self._log.warning(f"Rejected go_to_pose {args}: {error}")
            return False, error
        if self._move_to is None:
            return False, "go_to_pose is not available."
        if not self._busy.acquire(blocking=False):
            return False, "The arm is already busy with another job."
        try:
            if self._held is not None:
                return False, (
                    "The arm is holding a cube, so I won't move it to a named pose. "
                    + self._holding_message()
                )
            self._log.info(f"go_to_pose: {args['pose']} ({pose})")
            ok, message = self._move_to(pose)
            if not ok:
                return False, f"Could not move to {args['pose']}. {message}"
            return True, f"The arm is at the {args['pose']} pose."
        finally:
            self._busy.release()

    def _report_state(self, args):
        if args:
            return False, "report_state takes no arguments."
        view = self._look()
        if self._held is None:
            holding = "The arm is not holding anything."
        else:
            holding = f"The arm is holding the {NAME_OF[self._held]}."
        arm = "The arm is stopped." if self._stopped else "The arm is ready."
        busy = " The arm is busy with a job." if self._busy.locked() else ""
        seen = view.describe() if view is not None else NO_VIEW
        return True, f"{seen} {holding} {arm}{busy}"

    def run_tool(self, name, args):
        """Return (ok, message) for a tool call."""
        if name in ("stop_robot", "resume_robot"):
            return self._stop_or_resume(name, args)
        handlers = {
            "stack_cube": self._stack_cube,
            "pick_and_place": self._pick_and_place,
            "release_cube": self._release_cube,
            "go_to_pose": self._go_to_pose,
            "report_state": self._report_state,
        }
        if name not in handlers:
            return False, f"Unknown tool {name}."
        if self._stopped and name in MOVING_TOOLS:
            # A new command to move is a new decision: it ends the stop, no "resume"
            # needed. A stop heard while it runs still stops it.
            ok, message = self._stop_or_resume("resume_robot", {})
            if not ok:
                return False, f"The arm is stopped and could not resume. {message}"
            self._log.info(f"{name}: a new command ended the stop")
        return handlers[name](args)


def FakeExecutor(logger):  # noqa: N802
    """Dry run: every pick, place, stop, resume, release and move succeeds, and the
    cubes really move on a pretend table."""
    from colman_llm.fake_arm import FakeArm

    return FakeArm(logger).executor()
