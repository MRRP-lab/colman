"""What the cameras say is on the table, as the node uses it to plan.

The overhead camera (through the world_state node) is the only source of truth for
which cube sits on which. Nothing here remembers a move the arm made: a cube is on a
stack only if the latest /world_state says so. WorldView is one snapshot of that
message with the questions the executor asks of it. FakeWorld is a table the dry-run
mode and the tests move cubes around on.
"""

import math
from dataclasses import dataclass, field

from colman_llm.tools import CUBE_NAMES, STACKS

UNKNOWN = "unknown"
NAME_OF = {frame: name for name, frame in CUBE_NAMES.items()}
COLOR_OF = {f: c for c, frames in STACKS.items() for f in frames}
# stack_cube builds on the stack of a colour that sits nearest this distance from the
# arm base (m), the middle of what the arm reaches comfortably.
SWEET_SPOT_M = 0.30


@dataclass
class Column:
    tags: list  # bottom to top; UNKNOWN for a cube nobody has identified
    height: int
    order_known: bool = True
    top_seen: bool = True
    x: float = 0.0  # where the column is, in base_link (m)
    y: float = 0.0


@dataclass
class WorldView:
    columns: list = field(default_factory=list)
    held: str = ""
    missing: list = field(default_factory=list)

    @classmethod
    def from_msg(cls, msg):
        columns = [
            Column(
                list(s.tags),
                s.height,
                s.order_known,
                s.top_seen,
                s.position.x,
                s.position.y,
            )
            for s in msg.stacks
        ]
        return cls(columns, msg.held, list(msg.missing))

    # ------------------------------------------------------------------ queries
    def name_of(self, tag):
        return NAME_OF.get(tag, "an unidentified cube")

    def column_of(self, tag):
        return next((c for c in self.columns if tag in c.tags), None)

    def top_of(self, tag):
        column = self.column_of(tag)
        return column.tags[-1] if column else None

    @staticmethod
    def _is_pure(column, color):
        return all(t in STACKS[color] for t in column.tags)

    def anchor(self, color):
        """(column, reason): the stack of this colour the other cubes are put on.

        No cube is a base. It is the tallest column made only of this colour's cubes
        (a single cube counts), and among equals the one nearest the arm's sweet spot.
        """
        pure = [c for c in self.columns if self._is_pure(c, color)]
        if not pure:
            return None, f"I can't see any {color} cubes."
        tallest = max(c.height for c in pure)
        best = min(
            (c for c in pure if c.height == tallest),
            key=lambda c: abs(math.hypot(c.x, c.y) - SWEET_SPOT_M),
        )
        return best, None

    def height(self, color):
        """Cubes in the colour's stack (a single cube is 1), or 0 if none is in view."""
        column, _ = self.anchor(color)
        return column.height if column else 0

    # --------------------------------------------------------------- stack_cube
    def stack_done(self, color, count):
        return self.height(color) >= count

    def target_of(self, color):
        """(target, None): the top cube of the colour's stack, or (None, why not)."""
        column, reason = self.anchor(color)
        if column is None:
            return None, reason
        target = column.tags[-1]
        if target == UNKNOWN or not column.top_seen:
            return None, f"I can't see the top of the {color} stack."
        return target, None

    def next_move(self, color):
        """(cube, target, None), or (None, None, why not).

        The cube is a loose one of this colour, the one nearest the stack. Cubes in
        another stack are never taken apart.
        """
        target, reason = self.target_of(color)
        if target is None:
            return None, None, reason
        column, _ = self.anchor(color)
        loose = [
            c
            for c in self.columns
            if c is not column
            and c.height == 1
            and c.top_seen
            and c.tags[0] in STACKS[color]
        ]
        if not loose:
            return None, None, f"There are no loose {color} cubes left to stack."
        nearest = min(loose, key=lambda c: math.hypot(c.x - column.x, c.y - column.y))
        return nearest.tags[0], target, None

    # ----------------------------------------------------------- pick_and_place
    def check_move(self, cube, target):
        """Return None if cube can go on target, else why not, in plain language."""
        if cube not in NAME_OF:
            return "I don't know that cube."
        if cube == target:
            return "A cube cannot be placed on itself."
        own = self.column_of(cube)
        if own is None:
            return f"I can't see the {self.name_of(cube)}."
        if not own.top_seen:
            return f"I can't see the {self.name_of(cube)} right now."
        if own.tags[-1] != cube:
            above = own.tags[own.tags.index(cube) + 1]
            return (
                f"The {self.name_of(above)} is on top of the {self.name_of(cube)}, "
                "so it cannot be picked up yet."
            )
        dest = self.column_of(target)
        if dest is None:
            return f"I can't see the {self.name_of(target)}."
        if not dest.top_seen:
            return f"I can't see the top of the stack with the {self.name_of(target)}."
        if dest is own:
            return f"The {self.name_of(cube)} is already on the {self.name_of(target)}."
        if dest.tags[-1] != target:
            return (
                f"The {self.name_of(dest.tags[-1])} is already on top of the "
                f"{self.name_of(target)}."
            )
        return None

    def on_top_of(self, cube, target):
        """True when the camera sees cube as the top of the column that has target."""
        own = self.column_of(cube)
        dest = self.column_of(target)
        return own is not None and own is dest and own.tags[-1] == cube

    # ------------------------------------------------------------------- report
    def describe(self):
        """Plain-language account of the stacks and the loose cubes."""
        stacks, loose = [], []
        for column in self.columns:
            names = [self.name_of(t) for t in column.tags]
            if column.height > 1:
                note = "" if column.order_known else " (I am not sure of the order)"
                stacks.append(f"{' then '.join(names)}, bottom to top{note}")
            else:
                loose.append(names[0])
        lines = [
            "Stacks: " + "; ".join(stacks) + "." if stacks else "No cubes are stacked."
        ]
        lines.append(
            "Loose cubes on the table: " + ", ".join(loose) + "."
            if loose
            else "There are no loose cubes."
        )
        if self.missing:
            lines.append(
                "I can't see: " + ", ".join(self.name_of(t) for t in self.missing) + "."
            )
        return " ".join(lines)


class FakeWorld:
    """A table that holds cubes, for dry-run mode and the tests. Not the camera."""

    def __init__(self):
        self.columns = [[frame] for frame in sorted(NAME_OF)]
        # Where each cube first lay, a row across the table; a column stays where its
        # bottom cube is.
        self.spot = {
            frame: (0.30 + 0.02 * (i % 2), -0.21 + 0.06 * i)
            for i, frame in enumerate(sorted(NAME_OF))
        }
        self.held = ""

    def view(self):
        return WorldView(
            [
                Column(list(c), len(c), True, True, *self.spot[c[0]])
                for c in self.columns
            ],
            self.held,
            [],
        )

    def column(self, frame):
        return next(c for c in self.columns if frame in c)

    def pick(self, frame):
        column = self.column(frame)
        column.remove(frame)
        if not column:
            self.columns.remove(column)
        self.held = frame

    def place(self, target):
        self.column(target).append(self.held)
        self.held = ""

    def drop(self):
        if self.held:
            self.columns.append([self.held])
            self.held = ""
