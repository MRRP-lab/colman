"""What cubes are on the table and how they are stacked, worked out from tag poses.

Pure Python (no ROS) so every scenario can be tested. The overhead camera sees only
the top tag of a stack, so the height of a column comes from the top tag's corrected z:
level = round(z / 50 mm). A tag that vanishes under another tag at the same x, y is
remembered as covered; a tag that vanishes with nothing over it and the arm out of the
way is lost. When what the camera now sees contradicts what we remembered (a cube was
pulled from under a stack), the camera wins and the order is marked uncertain.
"""

import math
from dataclasses import dataclass, field

CUBE_M = 0.05  # a cube is 5 cm, so a tag on the top of level n is n * CUBE_M high
XY_TOL_M = 0.03  # tags closer than this in x, y are in the same column
COVER_M = 0.035  # a tag this much higher over another covers it
LOST_S = 2.0  # how long a tag may vanish with no explanation before it is lost
ARM_CLEAR_M = 0.12  # the arm hides the table within this of its base-to-tip line
UNKNOWN = "unknown"


def level_of(z):
    return max(1, round(z / CUBE_M))


def xy_gap(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class Column:
    tags: list  # bottom to top; UNKNOWN for a cube we cannot identify
    position: tuple  # x, y of the top tag
    height: int
    order_known: bool
    top_seen: bool  # False while the top tag is hidden by the arm


@dataclass
class Snapshot:
    columns: list = field(default_factory=list)
    missing: list = field(default_factory=list)
    lost: list = field(default_factory=list)  # tags whose remembered pose is stale

    def column_of(self, tag):
        return next((c for c in self.columns if tag in c.tags), None)


def clusters(poses):
    """Group tags whose x, y are within XY_TOL_M of each other (chained)."""
    groups = []
    for tag, pos in poses.items():
        near = [g for g in groups if any(xy_gap(pos, poses[t]) <= XY_TOL_M for t in g)]
        merged = [tag]
        for g in near:
            merged += g
            groups.remove(g)
        groups.append(merged)
    return groups


class StackModel:
    def __init__(self, all_tags):
        self.all_tags = list(all_tags)
        self.unexplained = {}  # tag -> when it first vanished with no explanation

    def covered(self, tag, pos, visible):
        return any(
            other != tag and xy_gap(pos, p) <= XY_TOL_M and p[2] >= pos[2] + COVER_M
            for other, p in visible.items()
        )

    def update(self, now_s, settled, fresh, held=None, arm_hides=lambda xy: False):
        """settled: tag -> (x, y, z) believed pose (seen or remembered).
        fresh: tag -> pose of tags the cameras see right now.
        held: the tag the arm is holding. arm_hides(xy): True if the arm is in the way.
        """
        beliefs = {t: p for t, p in settled.items() if t != held}
        visible = {t: p for t, p in fresh.items() if t != held}
        lost = []
        for tag, pos in beliefs.items():
            if tag in visible:
                self.unexplained.pop(tag, None)
            elif self.covered(tag, pos, visible) or arm_hides(pos[:2]):
                self.unexplained.pop(tag, None)
            else:
                since = self.unexplained.setdefault(tag, now_s)
                if now_s - since >= LOST_S:
                    lost.append(tag)
        for tag in list(self.unexplained):
            if tag not in beliefs or tag in lost:
                self.unexplained.pop(tag, None)
        members = {t: p for t, p in beliefs.items() if t not in lost}

        columns = []
        for group in clusters(members):
            column, dropped = self.column(group, members, visible)
            lost += dropped
            columns.append(column)
        columns.sort(key=lambda c: c.position)
        placed = {t for c in columns for t in c.tags}
        missing = [t for t in self.all_tags if t not in placed and t != held]
        return Snapshot(columns, missing, lost)

    def column(self, group, members, visible):
        seen = [t for t in group if t in visible]
        top = max(seen or group, key=lambda t: members[t][2])
        height = level_of(members[top][2])
        dropped = []
        slots = {}
        order_known = True
        # prefer tags we can see; the rest are remembered from before
        for tag in sorted(group, key=lambda t: (t not in visible, -members[t][2])):
            level = level_of(members[tag][2])
            if tag not in visible and seen and level >= height:
                dropped.append(tag)  # the stack is shorter than this tag says
                order_known = False
            elif level in slots:
                dropped.append(tag)
                order_known = False
            else:
                slots[level] = tag
        tags = [slots.get(level, UNKNOWN) for level in range(1, height + 1)]
        if UNKNOWN in tags:
            order_known = False
        return Column(tags, members[top][:2], height, order_known, bool(seen)), dropped
