"""What the cameras say is on the table, as the node uses it to plan.

The overhead camera (through the world_state node) is the only source of truth for
which cube sits on which. Nothing here remembers a move the arm made: a cube is on a
stack only if the latest /world_state says so. WorldView is one snapshot of that
message with the questions the executor asks of it. FakeWorld is a table the dry-run
mode and the tests move cubes around on.
"""

from dataclasses import dataclass, field

from colman_llm.tools import BASE_FRAMES, CUBE_NAMES, MAX_STACK, STACKS

UNKNOWN = "unknown"
NAME_OF = {frame: name for name, frame in CUBE_NAMES.items()}
COLOR_OF = {f: c for c, s in STACKS.items() for f in [s["base"], *s["cubes"]]}
MAX_HEIGHT = MAX_STACK + 1  # a base plus the cubes that go on top of it


@dataclass
class Column:
    tags: list  # bottom to top; UNKNOWN for a cube nobody has identified
    height: int
    order_known: bool = True
    top_seen: bool = True


@dataclass
class WorldView:
    columns: list = field(default_factory=list)
    held: str = ""
    missing: list = field(default_factory=list)

    @classmethod
    def from_msg(cls, msg):
        columns = [
            Column(list(s.tags), s.height, s.order_known, s.top_seen)
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

    def height(self, color):
        """Cubes on top of the colour's base, or 0 if the base is not in view."""
        column = self.column_of(STACKS[color]["base"])
        return column.height - 1 if column else 0

    def anchor(self, color):
        """(column, reason): the column the colour's cubes are stacked onto."""
        base = STACKS[color]["base"]
        column = self.column_of(base)
        if column is None:
            return None, f"I can't see the {color} base cube, so I can't stack on it."
        return column, None

    # --------------------------------------------------------------- stack_cube
    def target_of(self, color):
        """(target, None): the top cube of the colour's stack, or (None, why not)."""
        column, reason = self.anchor(color)
        if column is None:
            return None, reason
        if column.height >= MAX_HEIGHT:
            return None, f"The {color} stack is already full."
        target = column.tags[-1]
        if target == UNKNOWN or not column.top_seen:
            return None, f"I can't see the top of the {color} stack."
        return target, None

    def next_move(self, color):
        """(cube, target, None), or (None, None, why not)."""
        target, reason = self.target_of(color)
        if target is None:
            return None, None, reason
        column = self.column_of(target)
        for cube in STACKS[color]["cubes"]:
            own = self.column_of(cube)
            if own is not None and own is not column and own.tags == [cube]:
                return cube, target, None
        return None, None, f"There are no loose {color} cubes left to stack."

    # ----------------------------------------------------------- pick_and_place
    def check_move(self, cube, target):
        """Return None if cube can go on target, else why not, in plain language."""
        if cube in BASE_FRAMES or cube not in NAME_OF:
            return f"The {self.name_of(cube)} is a base and never moves."
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
        if dest.height + 1 > MAX_HEIGHT:
            return (
                f"That stack is already {MAX_HEIGHT} cubes high, so the "
                f"{self.name_of(target)} cannot take another."
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
                stacks.append(
                    f"the {names[0]} has {' then '.join(names[1:])} on top{note}"
                )
            elif column.tags[0] not in BASE_FRAMES:
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
        self.held = ""

    def view(self):
        return WorldView([Column(list(c), len(c)) for c in self.columns], self.held, [])

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
