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

CUBE_M = 0.05  # a cube is exactly 5 cm
# The table surface in base_link: the touched table marks average -0.006 and the MoveIt
# table box (colman_motion/world_scene.py) has its top here. A tag on the top of the
# cube at level n is exactly TABLE_TOP_Z + n * CUBE_M high.
TABLE_TOP_Z = -0.005
XY_TOL_M = 0.03  # tags closer than this in x, y are in the same column
LOST_S = 10.0  # how long a tag may vanish with no explanation before it is lost
# A cube placed on another can take several seconds to be detected, and its hand may
# linger over it, so a covered tag must outlast that before it is called lost.
ARM_HIDE_MAX_S = 30.0  # the arm may explain a missing tag for this long, no longer
CARRY_MEMORY_S = 60.0  # how long a stack's covered cubes are remembered for a slide
RESTORE_S = 8.0  # a deleted tag comes back if a cube lands on its spot within this
OUTAGE_S = 10.0  # after the camera was down this long, remembered cubes are doubted
# The sources of a tag pose that can still be trusted while the overhead camera is down.
TRUSTED_WITHOUT_OVERHEAD = ("wrist", "held")


def should_publish(state, camera_alive):
    """Whether a tag's pose may be published as tag_N.

    With the overhead camera down, a remembered or overhead pose is a guess that may be
    long out of date, and the motion server would act on it as if it were current. Only
    a pose the wrist camera sees right now, or a cube held in the gripper, is published.
    """
    return camera_alive or state in TRUSTED_WITHOUT_OVERHEAD


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
    return max(1, round((z - TABLE_TOP_Z) / CUBE_M))


def level_height(level):
    """The exact height of a tag on the top of the cube at this level."""
    return TABLE_TOP_Z + level * CUBE_M


def exact_heights(columns):
    """{tag: exact height} for every identified cube in the columns."""
    return {
        tag: level_height(i + 1)
        for column in columns
        for i, tag in enumerate(column.tags)
        if tag != UNKNOWN
    }


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
    restored: dict = field(
        default_factory=dict
    )  # tag -> pose, covered as it was dropped
    lost_why: dict = field(default_factory=dict)  # tag -> which rule dropped it

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
        self.prev_beliefs = {}  # the beliefs of the last update
        self.gone = {}  # tag -> (pose, when) of a belief dropped without being lost
        self.refused = {}  # top tag -> the last reason its slide was not carried
        self.why = {}  # tag -> which rule dropped it in the current update
        self.dead_since = None  # when the camera stopped, while it is down

    def covered(self, tag, pos, visible):
        """True if a seen tag sits over this one, at a higher cube level.

        Levels, not raw heights: a height reading wobbles by about 2 cm, which is
        enough to hide a cover that is exactly one cube higher.
        """
        return any(
            other != tag
            and xy_gap(pos, p) <= XY_TOL_M
            and level_of(p[2]) > level_of(pos[2])
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
        self.why = {}
        relocated, carried, notes = {}, {}, []
        if camera_alive:
            relocated, carried = self.carry(now_s, beliefs, visible, notes)
        self.uncertain -= set(visible)
        restored = self.restore(now_s, beliefs, visible, notes)
        if not camera_alive:
            self.unexplained.clear()
            self.arm_hidden.clear()
            if self.dead_since is None:
                self.dead_since = now_s
        elif self.dead_since is not None:
            self.camera_returned(now_s, beliefs, visible, notes)
        self.beneath = {
            top: v for top, v in self.beneath.items() if now_s - v[3] < CARRY_MEMORY_S
        }
        in_transit = self.in_transit(visible)
        for tag, pos in beliefs.items():
            if not camera_alive:
                break
            if tag in visible:
                self.unexplained.pop(tag, None)
                self.arm_hidden.pop(tag, None)
            elif tag in in_transit:
                # It was under a top that has gone out of sight, probably in a hand.
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
                    self.why[tag] = f"hidden by the arm for {now_s - since:.0f} s"
                    lost.append(tag)
            else:
                self.arm_hidden.pop(tag, None)
                since = self.unexplained.setdefault(tag, now_s)
                if now_s - since >= LOST_S:
                    self.why[tag] = (
                        f"not seen and not covered for {now_s - since:.0f} s"
                    )
                    lost.append(tag)
        for timers in (self.unexplained, self.arm_hidden):
            for tag in list(timers):
                if tag not in beliefs or tag in lost:
                    timers.pop(tag, None)
        members = {t: p for t, p in beliefs.items() if t not in lost}

        columns = []
        for group in clusters(members):
            column, dropped = self.column(group, members, visible, in_transit)
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
        self.note_gone(now_s, beliefs, lost, visible, held)
        placed = {t for c in columns for t in c.tags}
        missing = [t for t in self.all_tags if t not in placed and t != held]
        return Snapshot(
            columns,
            missing,
            lost,
            not camera_alive,
            relocated,
            notes,
            restored,
            dict(self.why),
        )

    def camera_returned(self, now_s, beliefs, visible, notes):
        """After a long outage, doubt what was only remembered.

        Cubes may have been moved, stacked or taken away while nothing was watching, so
        the order inside a stack, the stacks that had lost their top, and the cubes in
        transit are no longer something to rely on.
        """
        outage = now_s - self.dead_since
        self.dead_since = None
        if outage < OUTAGE_S:
            return
        self.uncertain.update(t for t in beliefs if t not in visible)
        self.ghosts = []
        self.beneath = {}
        notes.append(
            f"the camera was down {outage:.0f} s; remembered cubes are uncertain"
        )

    def restore(self, now_s, beliefs, visible, notes):
        """Bring back a tag dropped just before a cube landed on it.

        The tag under a cube being placed is sometimes seen at a slightly wrong pose
        in the moment before it is covered, and the node drops it as moved. If a
        taller tag then appears over where it was, it is under that cube.
        """
        restored = {}
        for tag, (pos, since) in list(self.gone.items()):
            if now_s - since > RESTORE_S or tag in visible or tag in beliefs:
                del self.gone[tag]
            elif self.covered(tag, pos, visible):
                beliefs[tag] = pos
                restored[tag] = pos
                self.uncertain.add(tag)
                notes.append(f"{tag} restored under the cube now over it")
                del self.gone[tag]
        return restored

    def note_gone(self, now_s, beliefs, lost, visible, held):
        """Remember tags whose belief disappeared without being lost or seen to move."""
        for tag, pos in self.prev_beliefs.items():
            if tag in beliefs or tag in lost or tag in visible or tag == held:
                continue
            self.gone.setdefault(tag, (pos, now_s))
        self.prev_beliefs = {t: p for t, p in beliefs.items() if t not in lost}

    def in_transit(self, visible):
        """Cubes remembered under a top that has left the stack, for up to a minute.

        A cube lifted away with the top of a stack stays hidden under it while the
        top is held or carried, so its absence says nothing yet. That holds whether
        the top is out of sight or seen in a hand away from the stack. It stops when
        the top is seen back at the old spot, with the stack intact, and the memory
        expires after a minute, when the normal wait for a missing tag applies.
        """
        return {
            tag
            for top, (slots, old_xy, _, _) in self.beneath.items()
            if not (top in visible and xy_gap(visible[top], old_xy) <= XY_TOL_M)
            for tag in slots
            if tag != UNKNOWN
        }

    def remember_beneath(self, now_s, columns):
        """Note which cubes are under each stack's visible top, in case it slides."""
        for column in columns:
            if not column.top_seen or column.height < 2:
                continue
            under = list(column.tags[:-1])  # bottom to top, UNKNOWN for a gap
            if any(t != UNKNOWN for t in under):
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
        """Move the remembered cubes under a stack's top when it is slid or lifted off.

        The settled top tag shows where it went, and how high it now sits says how many
        cubes went with it: a whole stack slid keeps its height, and the top two cubes
        lifted off together land two high, so one cube went along under the top. The
        cubes under it stay covered and are never seen to move, so they are carried
        only if the top is seen somewhere new, the spot is free, and each of them is
        still remembered, unseen, at the old spot. A top that landed alone or on
        another stack does not qualify. Returns the new poses of the carried cubes and
        {top: how many cubes were carried with it}.
        """
        relocated, carried = {}, {}
        notes = [] if notes is None else notes
        for top, (slots, old_xy, height, _) in list(self.beneath.items()):
            # Only a top that has settled counts. One seen moving, in a hand or in a
            # push, is at a height that says nothing about the cubes that went with it.
            if top not in beliefs or top not in visible:
                continue
            q = beliefs[top]
            if xy_gap(q, old_xy) <= XY_TOL_M:
                continue
            moved_cm = xy_gap(q, old_xy) * 100

            def refuse(why, top=top, moved_cm=moved_cm):
                if (
                    self.refused.get(top) != why
                ):  # say each reason once, not every frame
                    self.refused[top] = why
                    notes.append(f"{top} moved {moved_cm:.0f} cm, not carried: {why}")

            level = level_of(q[2])
            k = level - 1  # cubes that went with the top, in the stack under it
            if k < 1:
                refuse("it landed alone, nothing went with it")
                continue
            if k > len(slots):
                refuse(f"it sits higher than its stack was ({level} against {height})")
                continue
            moving = slots[-k:]
            under = [t for t in moving if t != UNKNOWN and t in beliefs]
            if not under:
                refuse("no cube under it is remembered")
                continue
            under = [t for t in under if t not in visible]  # a seen one tracks itself
            if not under:
                refuse("every cube under it is seen, so none needs carrying")
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
            for i, tag in enumerate(moving):
                if tag in under:
                    beliefs[tag] = (q[0], q[1], level_height(i + 1))
                    relocated[tag] = beliefs[tag]
                    self.unexplained.pop(tag, None)
                    self.arm_hidden.pop(tag, None)
            self.uncertain.update(under)
            self.refused.pop(top, None)
            carried[top] = k
            notes.append(f"{top} moved {moved_cm:.0f} cm, carried {', '.join(under)}")
            # Keep following this top. While it is still being pushed it has no settled
            # pose, so the carried cubes form a column with no top and nothing else
            # would remember that they belong under it.
            self.beneath[top] = (moving, tuple(q[:2]), k + 1, now_s)
        return relocated, carried

    def update_ghosts(self, now_s, columns, carried=None):
        """Keep a stack whose top was removed until its next cube is seen."""
        carried = carried or {}

        def near(a, others):
            return any(xy_gap(a, o) <= XY_TOL_M for o in others)

        positions = [c.position for c in columns]
        for prev in self.last_columns:
            if (
                prev.top_seen
                and prev.height >= 2
                and not near(prev.position, positions)
            ):
                if not near(prev.position, [g.position for g in self.ghosts]):
                    # the top left, and so did any cubes carried with it
                    tags = prev.tags[: -(carried.get(prev.tags[-1], 0) + 1)]
                    if tags:
                        self.ghosts.append(Ghost(tags, prev.position, len(tags), now_s))
        self.ghosts = [
            g
            for g in self.ghosts
            if now_s - g.since < GHOST_S and not near(g.position, positions)
        ]

    def column(self, group, members, visible, transit=frozenset()):
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
                # The stack is shorter than this tag says. If the tag went with a top
                # that was lifted away, keep it for the carry instead of losing it.
                if tag not in transit:
                    self.why[tag] = (
                        f"taller than its stack (level {level}, stack {height})"
                    )
                    dropped.append(tag)
                order_known = False
            elif level in slots:
                self.why[tag] = f"same level ({level}) as {slots[level]} in its column"
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
