"""Keep the cubes and stacks the overhead camera sees in MoveIt's planning scene.

Without this the planner knows only the table and the cube in the gripper, so a move
from one place to another can sweep the arm through a stack it cannot see. Each column
in /world_state becomes one box, 5 cm wide plus a margin and as tall as the stack.

The column being picked from or placed on is left out while that move runs: the arm
has to touch it, and a box there would make the move impossible. The cube in the
gripper is not in /world_state, so it only appears as the box attached to the tip.
"""

import threading
import time
from dataclasses import dataclass

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from colman_interfaces.msg import WorldState

CUBE_M = 0.05
# The top of the table in base_link: the table box added by the motion server has its
# top here, and the nozzle touched the table marks at about -0.006.
TABLE_TOP_Z = -0.005
# Added to each side. A tag is placed to a centimetre or two, so a tight box would
# let the arm graze a stack that is a little further over than it looks.
MARGIN_M = 0.01
MOVED_M = 0.01  # a box is rebuilt when its column shifts this far
STALE_S = 5.0  # a world state older than this is not current
SYNC_HZ = 2.0
FRAME = "base_link"
UNKNOWN = "unknown"
PREFIX = "cubes_"


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    height: int  # cubes high

    @property
    def dims(self):
        side = CUBE_M + 2 * MARGIN_M
        return [side, side, self.height * CUBE_M + MARGIN_M]

    @property
    def center(self):
        return (self.x, self.y, TABLE_TOP_Z + self.dims[2] / 2)


def column_id(stack):
    """A name for a column: its top cube when known, else where it stands."""
    top = stack.tags[-1]
    if top != UNKNOWN:
        return PREFIX + top
    return f"{PREFIX}at_{stack.position.x:.2f}_{stack.position.y:.2f}"


def wanted_boxes(stacks, excluded_tags=()):
    """{id: Box} for every column, except those holding an excluded tag."""
    excluded = set(excluded_tags)
    boxes = {}
    for stack in stacks:
        if excluded.intersection(stack.tags):
            continue
        boxes[column_id(stack)] = Box(stack.position.x, stack.position.y, stack.height)
    return boxes


def plan_updates(current, wanted):
    """(ids to remove, {id: Box} to add) to turn the scene from current into wanted."""
    remove, add = [], {}
    for obj_id, box in current.items():
        new = wanted.get(obj_id)
        if new is None or moved(box, new):
            remove.append(obj_id)
    for obj_id, box in wanted.items():
        if obj_id not in current or obj_id in remove:
            add[obj_id] = box
    return remove, add


def moved(old, new):
    shifted = max(abs(old.x - new.x), abs(old.y - new.y)) >= MOVED_M
    return shifted or old.height != new.height


class WorldSceneSync:
    """The current boxes and the rules for changing them. No ROS in here."""

    def __init__(self, scene, logger=None, clock=time.monotonic):
        self.scene = scene  # add_box(id, frame, dims, pos) and remove_object(id)
        self.log = logger
        self.clock = clock
        self.current = {}
        self.excluded = set()
        self.stacks = None
        self.stale = False
        self.heard_at = None
        self.lock = threading.Lock()
        self.warned_stale = False

    def on_world(self, msg):
        with self.lock:
            self.stacks = list(msg.stacks)
            self.stale = bool(msg.stale)
            self.heard_at = self.clock()

    def exclude(self, tags):
        """Leave the columns holding these tags out of the scene, at once."""
        with self.lock:
            self.excluded = {t for t in tags if t}
        self.sync()

    def clear_exclusions(self):
        with self.lock:
            self.excluded = set()
        self.sync()

    def current_world(self):
        """False until a world state has arrived, and when it is old or marked stale."""
        if self.heard_at is None:
            return False
        return not self.stale and self.clock() - self.heard_at < STALE_S

    def sync(self):
        """Apply the difference between the world state and the scene."""
        with self.lock:
            if self.stacks is None:
                return
            if not self.current_world() and not self.warned_stale:
                self.warned_stale = True
                self.warn("world state is stale: planning around the last known cubes")
            elif self.current_world():
                self.warned_stale = False
            # A stale world keeps the last known boxes: a cube that was there is
            # more likely still there than gone.
            wanted = wanted_boxes(self.stacks, self.excluded)
            remove, add = plan_updates(self.current, wanted)
            for obj_id in remove:
                self.scene.remove_object(obj_id)
                self.current.pop(obj_id, None)
            for obj_id, box in add.items():
                self.scene.add_box(obj_id, FRAME, box.dims, box.center)
                self.current[obj_id] = box

    def warn(self, text):
        if self.log is not None:
            self.log.warn(text)


class WorldScene(Node):
    """Feeds /world_state into the planning scene a couple of times a second."""

    def __init__(self, scene):
        super().__init__("world_scene")
        self.sync = WorldSceneSync(scene, self.get_logger())
        group = ReentrantCallbackGroup()
        self.create_subscription(
            WorldState, "/world_state", self.sync.on_world, 10, callback_group=group
        )
        self.create_timer(1.0 / SYNC_HZ, self.sync.sync, callback_group=group)

    def exclude(self, tags):
        self.sync.exclude(tags)

    def clear_exclusions(self):
        self.sync.clear_exclusions()

    def current_world(self):
        return self.sync.current_world()
