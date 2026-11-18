"""Crash-loop bookkeeping for restarting a process. No ROS or launch imports."""

from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class Decision:
    restart: bool
    delay: float
    crashes: int  # crashes seen inside the window, including this one


class CrashLoopGuard:
    """Decide whether a crashed process should be restarted, and after how long.

    Restarts back off exponentially (`delay`, 2x, 4x ... up to `max_delay`) while
    crashes keep arriving. After `max_crashes` crashes within `window` seconds it
    gives up, because a process that dies instantly every time is not going to be
    fixed by restarting it again, and a hot loop hides the real error in the logs.
    """

    def __init__(self, delay=2.0, max_delay=30.0, max_crashes=5, window=60.0):
        self.delay = delay
        self.max_delay = max_delay
        self.max_crashes = max_crashes
        self.window = window
        self._crashes = deque()
        self.gave_up = False

    def record_crash(self, now):
        if self.gave_up:
            return Decision(False, 0.0, len(self._crashes))
        self._crashes.append(now)
        while self._crashes and now - self._crashes[0] > self.window:
            self._crashes.popleft()
        count = len(self._crashes)
        if count >= self.max_crashes:
            self.gave_up = True
            return Decision(False, 0.0, count)
        return Decision(True, min(self.delay * 2 ** (count - 1), self.max_delay), count)
