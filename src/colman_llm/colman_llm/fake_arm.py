"""A pretend arm on a pretend table, for dry-run mode and the tests.

Every pick, place and release moves cubes on a FakeWorld, which stands in for what the
overhead camera would report. Results can be scripted to fail.
"""

import time

from colman_llm.executor import PickAndPlaceExecutor
from colman_llm.world_view import FakeWorld


class FakeArm:
    def __init__(self, logger, pick=None, place=None, release=None, move=None):
        self.log = logger
        self.world = FakeWorld()
        self.calls = []
        self._picks = list(pick or [])
        self._places = list(place or [])
        self._release = release or (True, "off")
        self._move = move or (True, "ok")

    def do_pick(self, frame):
        self.calls.append(("pick", frame))
        ok, message = self._picks.pop(0) if self._picks else (True, "ok")
        if ok:
            self.world.pick(frame)
        return ok, message

    def do_place(self, frame):
        self.calls.append(("place", frame))
        ok, message = self._places.pop(0) if self._places else (True, "ok")
        if ok:
            self.world.place(frame)
        return ok, message

    def do_release(self):
        self.calls.append(("release",))
        if self._release[0]:
            self.world.drop()
        return self._release

    def do_move(self, pose):
        self.calls.append(("move_to", pose))
        return self._move

    def executor(self, stop=True, sleep=lambda s: None, announce=None):
        return PickAndPlaceExecutor(
            self.do_pick,
            self.do_place,
            self.log,
            self.world.view,
            stop=(lambda: (True, "stopped")) if stop else None,
            resume=(lambda: (True, "resumed")) if stop else None,
            release=self.do_release,
            move_to=self.do_move,
            sleep=sleep if sleep is not None else time.sleep,
            announce=announce,
        )
