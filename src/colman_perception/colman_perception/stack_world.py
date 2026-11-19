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
LOST_S = 10.0  # how long a tag may vanish with no explanation before it is lost
# A cube placed on another can take several seconds to be detected, and its hand may
# linger over it, so a covered tag must outlast that before it is called lost.
ARM_HIDE_MAX_S = 30.0  # the arm may explain a missing tag for this long, no longer
CARRY_MEMORY_S = 60.0  # how long a stack's covered cubes are remembered for a slide
GHOST_S = 15.0  # how long a stack whose top was removed is kept while its tag is unseen
ARM_CLEAR_M = 0.12  # the arm hides the table within this of its base-to-tip line
UNKNOWN = "unknown"


# The overhead camera gets a small flat tag's height from the tag's apparent size and
# its pose solver sometimes flips between two answers 2 to 3 cm apart while x and y stay
# put. A tag has moved when it moves sideways, or its height changes by more than that
# swing (a cube lifted straight up is 5 cm).
SETTLE_XY_M = 0.003  # a tag is still while x and y stay this close
SETTLE_Z_M = 0.03  # ... and its height stays within the pose-flip swing
MOVED_XY_M = 0.02  # a tag has moved if it shifts this far sideways
MOVED_Z_M = 0.04  # ... or its height changes by this much


def shifted(a, b, xy_m, z_m):
    """True if poses a and b differ by xy_m sideways or z_m in height."""
    return math.hypot(a[0] - b[0], a[1] - b[1]) >= xy_m or abs(a[2] - b[2]) >= z_m


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
    stale: bool = False  # the camera is not delivering images, so nothing here is new
    relocated: dict = field(default_factory=dict)  # tag -> new pose, a slid stack
    notes: list = field(default_factory=list)  # why a slide was or was not carried

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


@dataclass
class Ghost:
    """What is left of a stack when its top cube was removed and nothing is seen yet."""

    tags: list
    position: tuple
    height: int
    since: float


class StackModel:
    def __init__(self, all_tags):
        self.all_tags = list(all_tags)
        self.unexplained = {}  # tag -> when it first vanished with no explanation
        self.arm_hidden = {}  # tag -> when the arm first started to explain its absence
        self.ghosts = []
        self.last_columns = []
        self.beneath = {}  # top tag -> (tags under it, column position, height, when)
        self.uncertain = set()  # tags placed by a slide guess, not seen to go there

    def covered(self, tag, pos, visible):
        return any(
            other != tag and xy_gap(pos, p) <= XY_TOL_M and p[2] >= pos[2] + COVER_M
            for other, p in visible.items()
        )

    def update(
        self,
        now_s,
        settled,
        fresh,
        held=None,
        arm_hides=lambda xy: False,
        camera_alive=True,
    ):
        """settled: tag -> (x, y, z) believed pose (seen or remembered).
        fresh: tag -> pose of tags the cameras see right now.
        held: the tag the arm is holding. arm_hides(xy): True if the arm is in the way.
        camera_alive: False while the overhead camera is not delivering images. Then
        nothing is learned from a missing tag, and the snapshot is marked stale.
        """
        beliefs = {t: p for t, p in settled.items() if t != held}
        visible = {t: p for t, p in fresh.items() if t != held}
        lost = []
        relocated, carried, notes = {}, set(), []
        if camera_alive:
            relocated, carried = self.carry(now_s, beliefs, visible, notes)
        self.uncertain -= set(visible)
        if not camera_alive:
            self.unexplained.clear()
            self.arm_hidden.clear()
        for tag, pos in beliefs.items():
            if not camera_alive:
                break
            if tag in visible:
                self.unexplained.pop(tag, None)
                self.arm_hidden.pop(tag, None)
            elif self.covered(tag, pos, visible):
                # A seen tag over it explains it for as long as that tag stays.
                self.unexplained.pop(tag, None)
                self.arm_hidden.pop(tag, None)
            elif arm_hides(pos[:2]):
                # The arm explains it only for a while: a memory that outlives the arm
                # would be a ghost at a place the cube may have left.
                self.unexplained.pop(tag, None)
                since = self.arm_hidden.setdefault(tag, now_s)
                if now_s - since >= ARM_HIDE_MAX_S:
                    lost.append(tag)
            else:
                self.arm_hidden.pop(tag, None)
                since = self.unexplained.setdefault(tag, now_s)
                if now_s - since >= LOST_S:
                    lost.append(tag)
        for timers in (self.unexplained, self.arm_hidden):
            for tag in list(timers):
                if tag not in beliefs or tag in lost:
                    timers.pop(tag, None)
        members = {t: p for t, p in beliefs.items() if t not in lost}

        columns = []
        for group in clusters(members):
            column, dropped = self.column(group, members, visible)
            lost += dropped
            columns.append(column)
        if camera_alive:
            self.update_ghosts(now_s, columns, carried)
        self.last_columns = list(columns)
        self.remember_beneath(now_s, columns)
        columns += [
            Column(g.tags, g.position, g.height, False, False) for g in self.ghosts
        ]
        columns.sort(key=lambda c: c.position)
        placed = {t for c in columns for t in c.tags}
        missing = [t for t in self.all_tags if t not in placed and t != held]
        return Snapshot(columns, missing, lost, not camera_alive, relocated, notes)

    def remember_beneath(self, now_s, columns):
        """Note which cubes are under each stack's visible top, in case it slides."""
        for column in columns:
            if not column.top_seen or column.height < 2:
                continue
            under = [t for t in column.tags[:-1] if t != UNKNOWN]
            if under:
                self.beneath[column.tags[-1]] = (
                    under,
                    column.position,
                    column.height,
                    now_s,
                )
        self.beneath = {
            top: v for top, v in self.beneath.items() if now_s - v[3] < CARRY_MEMORY_S
        }

    def carry(self, now_s, beliefs, visible, notes=None):
        """Move the remembered cubes under a stack's top when the stack slides.

        The top tag shows where the stack went, but the cubes under it stay covered
        and are never seen to move. They are carried only if the top is now seen
        somewhere new at the same height, the spot it has gone to is free, and every
        cube under it is still remembered, unseen, at the old spot. A top lifted off
        and put on the table, or on another stack, does not qualify. Returns the new
        poses of the carried cubes and the tops that carried them.
        """
        relocated, carried = {}, set()
        notes = [] if notes is None else notes
        for top, (under, old_xy, height, _) in list(self.beneath.items()):
            if top not in visible:
                continue
            q = visible[top]
            if xy_gap(q, old_xy) <= XY_TOL_M:
                continue
            moved_cm = xy_gap(q, old_xy) * 100

            def refuse(why, top=top, moved_cm=moved_cm):
                notes.append(f"{top} moved {moved_cm:.0f} cm, not carried: {why}")

            if any(t not in beliefs for t in under):
                refuse("a cube under it is no longer remembered")
                continue
            if level_of(q[2]) != height:
                refuse(f"its height changed from {height} to {level_of(q[2])}")
                continue
            if any(t in visible for t in under):
                refuse("a cube under it is now seen")
                continue
            if any(xy_gap(beliefs[t], old_xy) > XY_TOL_M for t in under):
                refuse("a cube under it is no longer at the old spot")
                continue
            crowded = any(
                other != top and other not in under and xy_gap(p, q) <= XY_TOL_M
                for other, p in {**beliefs, **visible}.items()
            )
            if crowded:
                refuse("another cube is at the new spot")
                continue
            for tag in under:
                z = beliefs[tag][2]
                beliefs[tag] = (q[0], q[1], z)
                relocated[tag] = beliefs[tag]
                self.unexplained.pop(tag, None)
                self.arm_hidden.pop(tag, None)
            self.uncertain.update(under)
            carried.add(top)
            notes.append(f"{top} moved {moved_cm:.0f} cm, carried {', '.join(under)}")
            del self.beneath[top]
        return relocated, carried

    def update_ghosts(self, now_s, columns, carried=()):
        """Keep a stack whose top was removed until its next cube is seen."""

        def near(a, others):
            return any(xy_gap(a, o) <= XY_TOL_M for o in others)

        positions = [c.position for c in columns]
        for prev in self.last_columns:
            if prev.tags[-1] in carried:
                continue  # the whole stack slid, nothing is left behind
            if (
                prev.top_seen
                and prev.height >= 2
                and not near(prev.position, positions)
            ):
                if not near(prev.position, [g.position for g in self.ghosts]):
                    tags = prev.tags[:-1]
                    self.ghosts.append(Ghost(tags, prev.position, len(tags), now_s))
        self.ghosts = [
            g
            for g in self.ghosts
            if now_s - g.since < GHOST_S and not near(g.position, positions)
        ]

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
        if UNKNOWN in tags or self.uncertain.intersection(tags):
            order_known = False
        return Column(
            tags, tuple(members[top][:2]), height, order_known, bool(seen)
        ), dropped
