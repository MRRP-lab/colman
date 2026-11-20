"""What the cameras say is on the table, as the node uses it to plan.

The overhead camera (through the world_state node) is the only source of truth for
which cube sits on which. Nothing here remembers a move the arm made: a cube is on a
stack only if the latest /world_state says so. WorldView is one snapshot of that
message with the questions the executor asks of it. FakeWorld is a table the dry-run
mode and the tests move cubes around on.
"""

from dataclasses import dataclass, field

from colman_description.workcell import cube_top_z

from colman_llm.tools import COLORS, CUBE_NAMES

UNKNOWN = "unknown"
NAME_OF = {frame: name for name, frame in CUBE_NAMES.items()}
COLOR_OF = {f: c for c, frames in COLORS.items() for f in frames}


@dataclass
class Column:
    tags: list  # bottom to top; UNKNOWN for a cube nobody has identified
    height: int
    order_known: bool = True
    top_seen: bool = True
    position: tuple = None  # (x, y) of the column in base_link, None if not known


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
                (s.position.x, s.position.y),
            )
            for s in msg.stacks
        ]
        return cls(columns, msg.held, list(msg.missing))

    # ------------------------------------------------------------------ queries
    def name_of(self, tag):
        return NAME_OF.get(tag, "an unidentified cube")

    def column_of(self, tag):
        return next((c for c in self.columns if tag in c.tags), None)

    def tag_point(self, tag):
        """(x, y, z) of the tag on top of this cube in base_link, or None if unknown.

        x and y are the column's. z is the top of the cube at its level in the column,
        counting from the table, because the cubes are exactly one size.
        """
        column = self.column_of(tag)
        if column is None or column.position is None:
            return None
        level = column.tags.index(tag) + 1
        return (*column.position, cube_top_z(level))

    def top_of(self, tag):
        column = self.column_of(tag)
        return column.tags[-1] if column else None

    def pure_columns(self, color):
        """Columns made only of cubes of this colour, tallest first, then by tag."""
        frames = set(COLORS[color])
        columns = [c for c in self.columns if c.tags and set(c.tags) <= frames]
        return sorted(columns, key=lambda c: (-c.height, min(c.tags)))

    def height(self, color):
        """Cubes in the tallest stack made only of this colour, or 0 if none."""
        columns = self.pure_columns(color)
        return columns[0].height if columns else 0

    # --------------------------------------------------------------- stack_cube
    def target_of(self, color):
        """(target, None): the top cube of the colour's tallest stack, or why not.

        No cube is a base: the stack is whichever column of this colour is tallest.
        """
        columns = self.pure_columns(color)
        if not columns:
            return None, f"I can't see any {color} cubes."
        column = columns[0]
        target = column.tags[-1]
        if target == UNKNOWN:
            return None, f"I can't see the top of the {color} stack."
        return target, None

    def next_move(self, color, count, can_pick=None):
        """(cube, target, None), or (None, None, why not), to grow the stack to count.

        The target is the top of the colour's tallest stack. The cube is the top of
        the shortest other stack made only of that colour, so loose cubes go first.
        can_pick(tag) says whether the arm can reach a cube; cubes it cannot reach are
        skipped.
        """
        columns = self.pure_columns(color)
        if not columns:
            return None, None, f"I can't see any {color} cubes."
        if columns[0].height >= count:
            return (
                None,
                None,
                (f"There is already a stack of {columns[0].height} {color} cubes."),
            )
        target, reason = self.target_of(color)
        if target is None:
            return None, None, reason
        sources = [c for c in columns[1:] if c.tags[-1] != UNKNOWN]
        if not sources:
            return None, None, f"There are no other free {color} cubes to add."
        if can_pick is not None:
            sources = [c for c in sources if can_pick(c.tags[-1])]
            if not sources:
                return (
                    None,
                    None,
                    (
                        f"None of the other free {color} cubes is within "
                        "the arm's reach."
                    ),
                )
        source = min(sources, key=lambda c: (c.height, min(c.tags)))
        return source.tags[-1], target, None

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
        if own.tags[-1] != cube:
            above = own.tags[own.tags.index(cube) + 1]
            return (
                f"The {self.name_of(above)} is on top of the {self.name_of(cube)}, "
                "so it cannot be picked up yet."
            )
        dest = self.column_of(target)
        if dest is None:
            return f"I can't see the {self.name_of(target)}."
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

    # Where each cube starts, in base_link: the white cubes in a row at x = 0.25 m and
    # the blue cubes behind them, all well inside what the arm can reach.
    START = {
        **{f: (0.25, y) for f, y in zip(COLORS["white"], (0.2, 0.07, -0.07, -0.2))},
        **{f: (0.35, y) for f, y in zip(COLORS["blue"], (0.2, 0.07, -0.07, -0.2))},
    }

    def __init__(self):
        self.columns = [[frame] for frame in sorted(NAME_OF)]
        self.xy = dict(self.START)  # position of each column, by its bottom cube
        self.held = ""
        self._held_xy = None

    def view(self):
        return WorldView(
            [Column(list(c), len(c), position=self.xy[c[0]]) for c in self.columns],
            self.held,
            [],
        )

    def column(self, frame):
        return next(c for c in self.columns if frame in c)

    def pick(self, frame):
        column = self.column(frame)
        self._held_xy = self.xy[column[0]]
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
            self.xy[self.held] = self._held_xy
            self.held = ""
