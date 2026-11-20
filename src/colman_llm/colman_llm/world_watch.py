"""Notices when the table changes by itself and says what changed.

The arm's own moves are not announced: while it is working, holding a cube or being
spoken to, the baseline is dropped, and it is set again once the table has been still
for a moment. After that, a change that lasts is announced once: a cube lost, found, or
moved by someone. A hand passing over the table does not last, so it is not announced.
"""

from dataclasses import dataclass, field

from colman_llm.world_view import UNKNOWN

SETTLE_S = 3.0  # how long the table must be still before it is the baseline
CONFIRM_S = 3.0  # how long a change must last before it is announced
MIN_GAP_S = 10.0  # least time between two announcements
NO_VIEW_S = 6.0  # how long without a view before saying the camera is lost
MAX_REPORTED = 3  # changes named in one announcement


@dataclass
class Snapshot:
    under: dict = field(default_factory=dict)  # cube -> the known cubes below it
    missing: frozenset = frozenset()

    @classmethod
    def of(cls, view):
        under = {}
        for column in view.columns:
            for i, tag in enumerate(column.tags):
                if tag != UNKNOWN:
                    below = tuple(t for t in column.tags[:i] if t != UNKNOWN)
                    under[tag] = below
        return cls(under, frozenset(view.missing))

    def key(self):
        return (frozenset(self.under.items()), self.missing)


def changes(before, after, name_of):
    """Plain-language sentences for what differs between two snapshots."""
    said = []
    lost = sorted(after.missing - before.missing)
    if lost:
        said.append(
            "I can't see the " + " or the ".join(name_of(t) for t in lost) + "."
        )
    found = sorted(before.missing - after.missing)
    if found:
        said.append(
            "I can see the " + " and the ".join(name_of(t) for t in found) + "."
        )
    for tag in sorted(before.under.keys() & after.under.keys()):
        was, now = before.under[tag], after.under[tag]
        if was == now:
            continue
        cube = name_of(tag)
        if now:
            said.append(f"Someone put the {cube} on the {name_of(now[-1])}.")
        else:
            said.append(f"Someone moved the {cube} off its stack.")
    return said


class WorldWatch:
    """update(view, now, busy) returns the sentences to say now, maybe none."""

    def __init__(self, name_of):
        self._name_of = name_of
        self._baseline = None
        self._last_key = None
        self._since = 0.0  # when the table last changed
        self._spoke_at = None
        self._no_view_since = None
        self._said_no_view = False

    def update(self, view, now, busy):
        if view is None:
            return self._without_view(now)
        said = self._with_view(now)
        self._no_view_since = None
        if busy or view.held:
            self._baseline = None  # the arm's own moves are not changes
            self._last_key = None
            return said
        snap = Snapshot.of(view)
        if snap.key() != self._last_key:
            self._last_key = snap.key()
            self._since = now
        if self._baseline is None:
            if now - self._since >= SETTLE_S:
                self._baseline = snap
            return said
        if snap.key() == self._baseline.key() or now - self._since < CONFIRM_S:
            return said
        if self._spoke_at is not None and now - self._spoke_at < MIN_GAP_S:
            return said
        said += self._announce(changes(self._baseline, snap, self._name_of), now)
        self._baseline = snap
        return said

    def _announce(self, sentences, now):
        if not sentences:
            return []
        if len(sentences) > MAX_REPORTED:
            sentences = sentences[:MAX_REPORTED] + ["Other things have changed too."]
        self._spoke_at = now
        return sentences

    def _without_view(self, now):
        self._last_key = None
        if self._no_view_since is None:
            self._no_view_since = now
        if self._said_no_view or now - self._no_view_since < NO_VIEW_S:
            return []
        self._said_no_view = True
        self._baseline = None
        return ["I can't see the table right now."]

    def _with_view(self, now):
        if not self._said_no_view:
            return []
        self._said_no_view = False
        return ["I can see the table again."]
