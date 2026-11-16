"""Which cube sits on which. Held by the node, not by Claude, so a misheard sentence or
trimmed chat history cannot corrupt it.

Every cube that can move is either on the table or on top of another cube, and the
bases never move. stack_cube and pick_and_place both go through this one model, so a
cube moved by a spoken pick_and_place is still where stack_cube expects it. The next
cube for a colour goes on the top of that colour's base stack.
"""

from colman_llm.tools import BASE_FRAMES, CUBE_NAMES, MAX_STACK, STACKS

NAME_OF = {frame: name for name, frame in CUBE_NAMES.items()}


class StackState:
    def __init__(self):
        # movable cube -> the cube it sits on (None: on the table)
        self._on = {f: None for s in STACKS.values() for f in s["cubes"]}

    # ------------------------------------------------------------------ queries
    def name_of(self, frame):
        return NAME_OF[frame]

    def cube_name(self, color, frame):
        """Spoken name, with the base as cube 1: 'white cube 2'."""
        frames = [STACKS[color]["base"], *STACKS[color]["cubes"]]
        return f"{color} cube {frames.index(frame) + 1}"

    def above(self, frame):
        """The cube sitting on frame, or None."""
        for cube, under in self._on.items():
            if under == frame:
                return cube
        return None

    def top_of(self, frame):
        """The top cube of the stack that frame is in."""
        while (up := self.above(frame)) is not None:
            frame = up
        return frame

    def depth(self, frame):
        """How many cubes are under frame in its stack; a base or table cube is 0."""
        depth = 0
        while (under := self._on.get(frame)) is not None:
            frame = under
            depth += 1
        return depth

    def height(self, color):
        """Number of cubes on top of the colour's base."""
        return self.depth(self.top_of(STACKS[color]["base"]))

    # --------------------------------------------------------------- stack_cube
    def next_move(self, color):
        """Return (cube_frame, target_frame) for the next cube, or None if none fits."""
        target = self.top_of(STACKS[color]["base"])
        if self.depth(target) >= MAX_STACK:
            return None
        for cube in STACKS[color]["cubes"]:
            if self._on[cube] is None and self.above(cube) is None:
                return cube, target
        return None

    def confirm(self, color):
        """Record that the cube from next_move was placed."""
        cube, target = self.next_move(color)
        self._on[cube] = target

    # ----------------------------------------------------------- pick_and_place
    def check_move(self, cube, target):
        """Return None if cube can go on target, else why not, in plain language."""
        if cube in BASE_FRAMES or cube not in self._on:
            return f"The {self.name_of(cube)} is a base and never moves."
        if cube == target:
            return "A cube cannot be placed on itself."
        covering = self.above(cube)
        if covering is not None:
            return (
                f"The {self.name_of(covering)} is on top of the {self.name_of(cube)}, "
                "so it cannot be picked up yet."
            )
        if self._on[cube] == target:
            return f"The {self.name_of(cube)} is already on the {self.name_of(target)}."
        covering = self.above(target)
        if covering is not None:
            return (
                f"The {self.name_of(covering)} is already on top of the "
                f"{self.name_of(target)}."
            )
        if self.depth(target) + 1 > MAX_STACK:
            return (
                f"That stack is already {MAX_STACK} cubes high, so the "
                f"{self.name_of(target)} cannot take another."
            )
        return None

    def move(self, cube, target):
        """Record that cube now sits on target."""
        self._on[cube] = target

    def drop(self, cube):
        """Record that cube was let go and now lies loose on the table."""
        self._on[cube] = None

    # ------------------------------------------------------------------- report
    def describe(self):
        """Plain-language account of the stacks and the loose cubes."""
        stacks, loose = [], []
        roots = [*BASE_FRAMES, *(c for c, under in self._on.items() if under is None)]
        for root in sorted(roots, key=lambda f: list(NAME_OF).index(f)):
            chain = []
            frame = root
            while (up := self.above(frame)) is not None:
                chain.append(self.name_of(up))
                frame = up
            if chain:
                stacks.append(
                    f"the {self.name_of(root)} has {' then '.join(chain)} on top"
                )
            elif root not in BASE_FRAMES:
                loose.append(self.name_of(root))
        lines = []
        lines.append(
            "Stacks: " + "; ".join(stacks) + "." if stacks else "No cubes are stacked."
        )
        lines.append(
            "Loose cubes on the table: " + ", ".join(loose) + "."
            if loose
            else "There are no loose cubes."
        )
        return " ".join(lines)
