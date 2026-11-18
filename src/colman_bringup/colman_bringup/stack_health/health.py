"""Pure logic for the stack health watchdog. No ROS imports.

The node feeds it a `Snapshot` of the ROS graph every tick. It tracks which
expected components are alive, debounces flicker and reports what changed, so
every transition can be unit tested with a fake graph and a fake clock.
"""

from dataclasses import dataclass, field
from enum import Enum, auto

NODE = "node"
SERVICE = "service"
ACTION = "action"

# The service that proves an action server is there.
ACTION_PROBE = "/_action/send_goal"


@dataclass(frozen=True)
class Component:
    kind: str
    name: str

    @property
    def label(self):
        return f"{self.kind}:{self.name}"


@dataclass
class Snapshot:
    """What the graph looks like right now. Names are fully qualified."""

    nodes: set = field(default_factory=set)
    services: set = field(default_factory=set)

    def has(self, c):
        if c.kind == NODE:
            return c.name in self.nodes
        if c.kind == SERVICE:
            return c.name in self.services
        return c.name + ACTION_PROBE in self.services


class Event(Enum):
    LOST = auto()  # was alive, now gone for longer than the grace
    RECOVERED = auto()  # came back after being reported missing
    NEVER_APPEARED = auto()  # not seen within the startup timeout


@dataclass(frozen=True)
class Change:
    event: Event
    component: Component


class Status(Enum):
    ALIVE = auto()
    STARTING = auto()  # never seen yet, still inside the startup timeout
    MISSING = auto()  # lost, or never appeared


class HealthMonitor:
    def __init__(self, expected, grace=3.0, startup_timeout=60.0):
        self.expected = list(expected)
        self.grace = grace
        self.startup_timeout = startup_timeout
        self._started = None
        self._seen = set()
        self._missing_since = {}
        self._reported_missing = set()
        self.status = {c: Status.STARTING for c in self.expected}

    def update(self, snap, now):
        if self._started is None:
            self._started = now
        changes = []
        for c in self.expected:
            if snap.has(c):
                self._missing_since.pop(c, None)
                self._seen.add(c)
                self.status[c] = Status.ALIVE
                if c in self._reported_missing:
                    self._reported_missing.discard(c)
                    changes.append(Change(Event.RECOVERED, c))
            elif c in self._seen:
                since = self._missing_since.setdefault(c, now)
                # Inside the grace the last known status stands.
                if now - since >= self.grace:
                    self.status[c] = Status.MISSING
                    if c not in self._reported_missing:
                        self._reported_missing.add(c)
                        changes.append(Change(Event.LOST, c))
            elif now - self._started >= self.startup_timeout:
                self.status[c] = Status.MISSING
                if c not in self._reported_missing:
                    self._reported_missing.add(c)
                    changes.append(Change(Event.NEVER_APPEARED, c))
        return changes

    def summary(self):
        """Labels of (missing, still starting) components."""
        missing = [c.label for c in self.expected if self.status[c] == Status.MISSING]
        starting = [c.label for c in self.expected if self.status[c] == Status.STARTING]
        return missing, starting


class Reaction(Enum):
    SAY = auto()
    RELEASE_VACUUM = auto()
    STOP_MOTION = auto()


FRIENDLY = {
    "motion_server": "the motion server",
    "llm_node": "the language model",
    "kokoro_tts": "speech output",
    "speech_to_text": "speech input",
    "driver_supervisor": "the arm supervisor",
    "camera_publisher": "the camera",
    "pick": "the pick action",
    "place": "the place action",
    "move_to": "the move action",
    "stop": "the stop service",
    "resume": "the resume service",
}


def _friendly(c):
    base = c.name.rsplit("/", 1)[-1]
    return FRIENDLY.get(base, base)


def speech_for(changes):
    """One short sentence for a batch of changes, or "" if nothing to say.

    When a node dies, its services and actions vanish with it, so only the
    nodes are named in that case.
    """
    lost = [ch.component for ch in changes if ch.event != Event.RECOVERED]
    back = [ch.component for ch in changes if ch.event == Event.RECOVERED]

    def names(components):
        nodes = [c for c in components if c.kind == NODE]
        return sorted({_friendly(c) for c in (nodes or components)})

    parts = []
    if lost:
        parts.append(f"Warning: {' and '.join(names(lost))} stopped responding.")
    if back:
        parts.append(f"{' and '.join(names(back))} is back.")
    return " ".join(parts)


def plan_reactions(
    changes,
    motion_node,
    stop_service,
    release_vacuum_on_loss=True,
    stop_on_motion_return=True,
):
    """Turn changes into `(Reaction, text)` pairs; the node carries them out.

    The motion server is the one component with a safety consequence. When it
    dies the vacuum can stay on, so release it. When it comes back it starts
    accepting goals at once, so stop it until a person says resume.
    """
    out = []
    for ch in changes:
        if (
            ch.event != Event.RECOVERED
            and release_vacuum_on_loss
            and ch.component == motion_node
        ):
            out.append((Reaction.RELEASE_VACUUM, ""))
        if (
            ch.event == Event.RECOVERED
            and stop_on_motion_return
            and ch.component == stop_service
        ):
            out.append((Reaction.STOP_MOTION, ""))
    text = speech_for(changes)
    if text:
        out.append((Reaction.SAY, text))
    return out
