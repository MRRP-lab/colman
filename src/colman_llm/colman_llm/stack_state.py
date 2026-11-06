"""Which cubes are stacked. Held by the node, not by Claude, so a misheard sentence or
trimmed chat history cannot corrupt it.

Each colour has a base cube that never moves and a list of loose cubes. The next cube
goes on the top of the stack, which is the base or the last cube placed.
"""

from colman_llm.tools import STACKS


class StackState:
    def __init__(self):
        self._stacked = {color: [] for color in STACKS}

    def next_move(self, color):
        """Return (cube_frame, target_frame) for the next cube, or None if full."""
        stacked = self._stacked[color]
        loose = STACKS[color]["cubes"]
        if len(stacked) >= len(loose):
            return None
        target = stacked[-1] if stacked else STACKS[color]["base"]
        return loose[len(stacked)], target

    def confirm(self, color):
        """Record that the cube from next_move was placed."""
        cube, _ = self.next_move(color)
        self._stacked[color].append(cube)

    def height(self, color):
        """Number of cubes on top of the base."""
        return len(self._stacked[color])

    def cube_name(self, color, frame):
        """Spoken name, with the base as cube 1: 'white cube 2'."""
        frames = [STACKS[color]["base"], *STACKS[color]["cubes"]]
        return f"{color} cube {frames.index(frame) + 1}"
