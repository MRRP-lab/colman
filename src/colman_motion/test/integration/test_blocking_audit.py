"""Static audit: blocking calls in the motion server's code must have a timeout.

A call that waits for another process (a trajectory controller, a service, a thread)
with no bound turns that process dying into the motion server hanging with its goal
EXECUTING forever. This test needs no ROS stack. It walks the server's modules and
reports each such call that is not on the reviewed list below, with the reason it is
safe. The behavioural counterpart is test_fault_injection.py.
"""

import ast
from pathlib import Path

import pytest

import colman_motion

SERVER_MODULES = [
    "motion_server.py",
    "arm_control.py",
    "scene_manager.py",
    "tag_lookup.py",
    "vacuum_control.py",
    "scan.py",
    "robot_ready.py",
]

# Methods that block until something else happens.
BLOCKING = {
    "wait",
    "wait_for_execution",
    "wait_for_service",
    "wait_for_server",
    "wait_for_current_robot_state",
    "spin_until_future_complete",
    "join",
    "result",
    "acquire",
    "execute_and_wait",
}
BOUND_KEYWORDS = {"timeout", "timeout_sec", "timeout_s"}

# (module, enclosing function, method) -> why an unbounded call is fine there.
REVIEWED = {
    ("arm_control.py", "execute_once", "wait_for_execution"): (
        "only after the completion callback has fired, so the trajectory is over"
    ),
    ("vacuum_control.py", "set_digital_out", "result"): (
        "only after future.done() is true, in a loop bounded by a deadline"
    ),
}


def calls_without_a_bound(path):
    tree = ast.parse(path.read_text())
    found = []

    class Visitor(ast.NodeVisitor):
        def __init__(self):
            self.function = "<module>"

        def visit_FunctionDef(self, node):
            outer, self.function = self.function, node.name
            self.generic_visit(node)
            self.function = outer

        visit_AsyncFunctionDef = visit_FunctionDef

        def visit_Call(self, node):
            name = getattr(node.func, "attr", None)
            if name in BLOCKING:
                bounded = bool(node.args) or any(
                    kw.arg in BOUND_KEYWORDS for kw in node.keywords
                )
                # str.join(iterable) and dict.get-style calls take an argument.
                if not bounded:
                    found.append((path.name, self.function, name, node.lineno))
            self.generic_visit(node)

    Visitor().visit(tree)
    return found


def all_unbounded():
    package = Path(colman_motion.__file__).parent
    found = []
    for module in SERVER_MODULES:
        found += calls_without_a_bound(package / module)
    return found


@pytest.mark.xfail(
    strict=False,
    reason=(
        "BUG unbounded-execute-wait: ArmControl.execute_once waits for the "
        "trajectory with Event.wait() and wait_for_execution() and no timeout, so a "
        "controller that never reports back leaves the goal EXECUTING forever. "
        "Fixed by the 'Bound the trajectory wait' commit on motion-fixes."
    ),
)
def test_no_unreviewed_blocking_call_is_unbounded():
    offenders = [
        f"{module}:{line} {function}() .{method}()"
        for module, function, method, line in all_unbounded()
        if (module, function, method) not in REVIEWED
    ]
    assert not offenders, "blocking calls without a timeout:\n" + "\n".join(offenders)


def test_reviewed_list_has_no_dead_entries():
    live = {(m, f, c) for m, f, c, _ in all_unbounded()}
    stale = sorted(set(REVIEWED) - live)
    assert not stale, f"entries that no longer match any call, remove them: {stale}"
