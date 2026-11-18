"""Launch helper that restarts a crashed process, with a crash-loop guard.

`respawn=True` on a launch Node restarts forever at a fixed rate. That hides a
node that dies instantly every time and keeps hammering whatever it talks to. Use
`supervise` instead: it restarts with exponential backoff and, after too many
crashes in a short window, gives up and logs loudly.

    supervise("llm_node", lambda: [Node(package="colman_llm", executable="llm_node")])

`match` is a prefix of the launch process name (the executable name for a Node,
the `name=` of an ExecuteProcess). `factory` returns the actions that (re)start
exactly that process, so it can also be an IncludeLaunchDescription of a launch
file holding a single node.
"""

import time

from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

from colman_bringup.crash_guard import CrashLoopGuard


def supervise(
    match,
    factory,
    delay=2.0,
    max_delay=30.0,
    max_crashes=5,
    window=60.0,
    clock=time.monotonic,
):
    """Return the actions that start `factory()` and keep it running."""
    guard = CrashLoopGuard(delay, max_delay, max_crashes, window)

    def on_exit(event, context):
        if not event.process_name.startswith(match):
            return None
        if context.is_shutdown:
            return None
        decision = guard.record_crash(clock())
        why = f"{match} exited with code {event.returncode}"
        if not decision.restart:
            return [
                LogInfo(
                    msg=(
                        f"[supervise] {why}; {decision.crashes} crashes in "
                        f"{window:.0f}s, GIVING UP. Not restarting it. Fix the cause "
                        "and relaunch."
                    )
                )
            ]
        return [
            LogInfo(
                msg=(
                    f"[supervise] {why} (crash {decision.crashes}/{max_crashes}); "
                    f"restarting in {decision.delay:.1f}s"
                )
            ),
            TimerAction(period=decision.delay, actions=list(factory())),
        ]

    return [
        *factory(),
        RegisterEventHandler(OnProcessExit(on_exit=on_exit)),
    ]
