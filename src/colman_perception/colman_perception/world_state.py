import math
from collections import deque
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from ur_msgs.msg import IOStates

from colman_interfaces.msg import WorldStack
from colman_interfaces.msg import WorldState as WorldStateMsg
from colman_perception.overhead_correction import correct_position
from colman_perception.stack_world import (
    ARM_CLEAR_M,
    CUBE_M,
    SETTLE_XY_M,
    SETTLE_Z_M,
    StackModel,
    exact_heights,
    level_height,
    moved_away,
    shifted,
    should_publish,
)

BASE_FRAME = "base_link"
TIP_FRAME = "vacuum_tip_link"
OVERHEAD_FRAME = "overhead_camera_optical_frame"
TAG_HEIGHT_M = 0.04  # about where a cube tag is, for the arm's shadow
ARM_LINK_FRAMES = ("forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link")
WRIST_CAMERA_FRAME = "oakd_pro_camera_rgb_optical_frame_wrist"
APRILTAG_CONFIGS = ["apriltag.yaml", "apriltag_overhead.yaml"]
SOURCES = ["wrist", "overhead"]
PREFIX = {"wrist": "wrist_", "overhead": "ovh_"}
TOLERANCE_M = 0.003
FRESH_S = {"wrist": 0.3, "overhead": 2.0}  # 4K detection runs at about 2 Hz
SETTLE_S = 1.0
MOVED_M = 0.02
GRAB_M = 0.03
DISAGREE_M = 0.010
DISAGREE_LOG_S = 5.0
STILL_M = 0.001
STILL_S = 0.3
RATE_HZ = 30.0
IO_TOPIC = "/io_and_status_controller/io_states"
JOINT_TOPIC = "/joint_states"
ARM_ALIVE_S = 2.0  # the arm position is only trusted while joint states keep arriving
WORLD_TOPIC = "/world_state"
DETECTIONS_TOPIC = "/detections"
OVERHEAD_FRAME_HINT = "overhead"  # the frame_id of the overhead detector's output
CAMERA_ALIVE_S = 5.0  # the overhead detector counts as stopped after this long
PUBLISH_EVERY = 3  # ticks of the update timer between /world_state messages
VACUUM_PIN = 0


def load_tag_frames():
    config = Path(get_package_share_directory("colman_perception")) / "config"
    ids = set()
    for name in APRILTAG_CONFIGS:
        for node in yaml.safe_load((config / name).read_text()).values():
            ids.update(node["ros__parameters"]["tag"]["ids"])
    return [f"tag_{i}" for i in sorted(ids)]


def seconds(duration):
    return duration.nanoseconds / 1e9


def moved(a, b, tolerance_m):
    return max(abs(p - q) for p, q in zip(a, b)) >= tolerance_m


def mean(samples):
    return [sum(s[1][i] for s in samples) / len(samples) for i in range(3)]


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q, v):
    return quat_mul(quat_mul(q, (*v, 0.0)), quat_conj(q))[:3]


def position_of(transform):
    t = transform.transform.translation
    return (t.x, t.y, t.z)


def rotation_of(transform):
    r = transform.transform.rotation
    return (r.x, r.y, r.z, r.w)


class WorldState(Node):
    def __init__(self):
        super().__init__("world_state")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.tag_frames = load_tag_frames()
        self.since = {}
        self.samples = {}
        self.belief = {}
        self.state = {}
        self.camera_track = deque()
        self.held = None
        self.vacuum_on = None
        self.exact_z = {}  # tag -> the exact height of its level, from the last update
        self.hides = lambda xy: False  # True where the arm is over the table, as of now
        self.model = StackModel(self.tag_frames)
        self.world_pub = self.create_publisher(WorldStateMsg, WORLD_TOPIC, 10)
        self.ticks = 0
        self.camera_seen = None
        self.joints_seen = None
        self.create_subscription(JointState, JOINT_TOPIC, self.on_joints, 10)
        # The detector publishes an array for every image it processes, even an empty
        # one, so these arriving means the camera and detector are both running.
        self.create_subscription(
            AprilTagDetectionArray, DETECTIONS_TOPIC, self.on_detections, 10
        )
        self.create_subscription(IOStates, IO_TOPIC, self.on_io, 10)
        self.create_timer(1.0 / RATE_HZ, self.update)
        self.get_logger().info(f"world state tracking {', '.join(self.tag_frames)}")

    def lookup(self, frame):
        try:
            return self.buffer.lookup_transform(BASE_FRAME, frame, Time())
        except TransformException:
            return None

    def forget(self, key):
        self.since.pop(key, None)
        self.samples.pop(key, None)

    def arm_still(self, now):
        camera = self.lookup(WRIST_CAMERA_FRAME)
        if camera is None:
            self.camera_track.clear()
            return False
        position = position_of(camera)
        track = self.camera_track
        track.append((now, position))
        while len(track) > 1 and seconds(now - track[1][0]) >= STILL_S:
            track.popleft()
        if seconds(now - track[0][0]) < STILL_S:
            return False
        return not any(moved(p, position, STILL_M) for _, p in track)

    def observe(self, tag_frame, source, now, usable):
        key = (tag_frame, source)
        transform = self.lookup(PREFIX[source] + tag_frame) if usable else None
        if transform is None:
            self.forget(key)
            return None, None
        stamp = Time.from_msg(transform.header.stamp)
        if seconds(now - stamp) > FRESH_S[source]:
            self.forget(key)
            return None, None

        position = position_of(transform)
        if source == "overhead":
            position = correct_position(position)
        samples = self.samples.setdefault(key, deque())
        if not samples or stamp > samples[-1][0]:
            if samples and shifted(position, mean(samples), SETTLE_XY_M, SETTLE_Z_M):
                samples.clear()
            if not samples:
                self.since[key] = stamp
            samples.append((stamp, position, rotation_of(transform)))
            while seconds(stamp - samples[0][0]) > SETTLE_S:
                samples.popleft()

        if seconds(samples[-1][0] - self.since[key]) < SETTLE_S:
            return position, None
        return position, (mean(samples), samples[-1][2])

    def on_joints(self, msg):
        self.joints_seen = self.get_clock().now()

    def arm_known(self, now):
        """True only while a real arm is reporting joints.

        robot_state_publisher publishes the arm at its zero pose when nothing sends
        joint states, and that pose is not where the arm is.
        """
        return (
            self.joints_seen is not None
            and seconds(now - self.joints_seen) < ARM_ALIVE_S
        )

    def on_detections(self, msg):
        if OVERHEAD_FRAME_HINT in msg.header.frame_id:
            self.camera_seen = self.get_clock().now()

    def camera_alive(self, now):
        return (
            self.camera_seen is not None
            and seconds(now - self.camera_seen) < CAMERA_ALIVE_S
        )

    def on_io(self, msg):
        on = next(
            (d.state for d in msg.digital_out_states if d.pin == VACUUM_PIN), None
        )
        if on is None or on == self.vacuum_on:
            return
        self.vacuum_on = on
        if on:
            self.grab()
        else:
            self.drop()

    def believed_position(self, tag_frame):
        """Where the cube is believed to be, at the exact height of its level.

        The measured height is good to a centimetre or two and the arm is sent to the
        exact one, so a grasp has to be judged against the exact one too.
        """
        position = list(self.belief[tag_frame][0])
        if tag_frame in self.exact_z:
            position[2] = self.exact_z[tag_frame]
        return position

    def grab(self):
        tip = self.lookup(TIP_FRAME)
        if tip is None or not self.belief:
            self.get_logger().warn("vacuum on, no cube known near the tip")
            return
        tip_position = position_of(tip)
        tag_frame = min(
            self.belief,
            key=lambda t: math.dist(self.believed_position(t), tip_position),
        )
        position = self.believed_position(tag_frame)
        gap_m = math.dist(position, tip_position)
        if gap_m > GRAB_M:
            self.get_logger().warn(
                f"vacuum on, nearest cube {tag_frame} is {gap_m * 1000:.0f} mm away"
            )
            return
        rotation = self.belief[tag_frame][1]
        tip_inverse = quat_conj(rotation_of(tip))
        offset = rotate(tip_inverse, [p - q for p, q in zip(position, tip_position)])
        self.held = (tag_frame, offset, quat_mul(tip_inverse, rotation))
        self.get_logger().info(f"holding {tag_frame}")

    def drop(self):
        if self.held is None:
            return
        tag_frame = self.held[0]
        pose = self.held_pose()
        self.held = None
        # The cube is where the arm let go of it. Remember that, so a cube the arm still
        # hides is not lost; the camera corrects it, or drops it, once it can see.
        if pose is not None:
            position = list(pose[0])
            # A cube let go of in the air falls onto whatever is below it, so it is
            # there, not at the tip: a box at the tip would block every plan.
            below = max(
                (
                    len([t for t in column.tags if t != tag_frame])
                    for column in self.model.last_columns
                    if math.dist(column.position, position[:2]) < CUBE_M
                ),
                default=0,
            )
            position[2] = min(position[2], level_height(below + 1))
            self.belief[tag_frame] = (tuple(position), pose[1])
        else:
            self.belief.pop(tag_frame, None)
        for source in SOURCES:
            self.forget((tag_frame, source))
        self.get_logger().info(f"released {tag_frame}")

    def held_pose(self):
        tip = self.lookup(TIP_FRAME)
        if tip is None:
            return None
        _, offset, rotation_offset = self.held
        tip_rotation = rotation_of(tip)
        shifted = rotate(tip_rotation, offset)
        position = [p + m for p, m in zip(position_of(tip), shifted)]
        return position, quat_mul(tip_rotation, rotation_offset)

    def check_disagreement(self, tag_frame, wrist, overhead):
        gap_m = math.dist(wrist[0], overhead[0])
        if gap_m > DISAGREE_M:
            self.get_logger().warn(
                f"{tag_frame}: overhead is {gap_m * 1000:.1f} mm from wrist",
                throttle_duration_sec=DISAGREE_LOG_S,
            )

    def track(self, tag_frame, now, wrist_usable):
        if self.held is not None and self.held[0] == tag_frame:
            return "held", self.held_pose(), None

        usable = {"wrist": wrist_usable, "overhead": True}
        fresh = {}
        settled = {}
        for source in SOURCES:
            fresh[source], settled[source] = self.observe(
                tag_frame, source, now, usable[source]
            )
        if settled["wrist"] is not None and settled["overhead"] is not None:
            self.check_disagreement(tag_frame, settled["wrist"], settled["overhead"])

        seen = next((p for p in fresh.values() if p is not None), None)
        source = next((s for s in SOURCES if settled[s] is not None), None)
        if source is not None:
            self.belief[tag_frame] = settled[source]
            return source, settled[source], seen

        belief = self.belief.get(tag_frame)
        if belief is None:
            return None, None, seen
        if moved_away(fresh.values(), belief[0], self.hides(belief[0][:2])):
            del self.belief[tag_frame]
            return None, None, seen
        return "remembered", belief, seen

    def publish(self, tag_frame, pose, now):
        position, rotation = pose
        msg = TransformStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.child_frame_id = tag_frame
        t = msg.transform.translation
        t.x, t.y, t.z = position
        r = msg.transform.rotation
        r.x, r.y, r.z, r.w = rotation
        self.broadcaster.sendTransform(msg)

    def arm_hides(self, tip):
        """True if (x, y) is in the arm's shadow, as the overhead camera sees it.

        The arm is a polyline through its joints. The camera is above and to one side
        of the table, so each joint hides the table where the ray from the camera
        through it lands, not straight below it.
        """
        if tip is None:
            return lambda xy: False  # no robot model, so no arm in the way
        camera = self.lookup(OVERHEAD_FRAME)
        eye = position_of(camera) if camera is not None else None
        joints = [(0.0, 0.0, 0.0)]
        for frame in ARM_LINK_FRAMES:
            link = self.lookup(frame)
            if link is not None:
                joints.append(tuple(position_of(link)))
        joints.append(tuple(tip[:3]))

        def shadow(joint):
            if eye is None or eye[2] - joint[2] < 0.2:
                return joint[:2]
            k = (eye[2] - TAG_HEIGHT_M) / (eye[2] - joint[2])
            return (eye[0] + (joint[0] - eye[0]) * k, eye[1] + (joint[1] - eye[1]) * k)

        points = [shadow(j) for j in joints]
        segments = list(zip(points, points[1:]))

        def hides(xy):
            for (x0, y0), (x1, y1) in segments:
                dx, dy = x1 - x0, y1 - y0
                length = dx * dx + dy * dy
                along = 0.0
                if length > 1e-9:
                    along = ((xy[0] - x0) * dx + (xy[1] - y0) * dy) / length
                    along = max(0.0, min(1.0, along))
                gap = math.hypot(xy[0] - x0 - along * dx, xy[1] - y0 - along * dy)
                if gap < ARM_CLEAR_M:
                    return True
            return False

        return hides

    def publish_world(self, snapshot, now):
        msg = WorldStateMsg()
        msg.stamp = now.to_msg()
        msg.held = self.held[0] if self.held is not None else ""
        msg.stale = snapshot.stale
        msg.missing = list(snapshot.missing)
        for column in snapshot.columns:
            stack = WorldStack()
            stack.tags = list(column.tags)
            stack.position.x, stack.position.y = column.position
            stack.height = column.height
            stack.order_known = column.order_known
            stack.top_seen = column.top_seen
            msg.stacks.append(stack)
        self.world_pub.publish(msg)

    def update(self):
        now = self.get_clock().now()
        wrist_usable = self.arm_still(now)
        camera_alive = self.camera_alive(now)
        poses, states, settled, fresh = {}, {}, {}, {}
        tip = self.lookup(TIP_FRAME)
        known = tip is not None and self.arm_known(now)
        hides = self.arm_hides(position_of(tip) if known else None)
        self.hides = hides
        for tag_frame in self.tag_frames:
            state, pose, seen = self.track(tag_frame, now, wrist_usable)
            if state != self.state.get(tag_frame):
                self.get_logger().info(f"{tag_frame}: {state or 'unknown'}")
                self.state[tag_frame] = state
            if pose is not None:
                poses[tag_frame] = pose
                states[tag_frame] = state
                if state != "held":
                    settled[tag_frame] = pose[0]
            if seen is not None:
                fresh[tag_frame] = seen

        held = self.held[0] if self.held is not None else None
        snapshot = self.model.update(
            now.nanoseconds / 1e9,
            settled,
            fresh,
            held,
            hides,
            camera_alive,
        )
        for tag_frame, pose in snapshot.restored.items():
            if self.belief.get(tag_frame) is None:
                self.belief[tag_frame] = (list(pose), (0.0, 0.0, 0.0, 1.0))
        for note in snapshot.notes:
            self.get_logger().info(note, throttle_duration_sec=0.5)
        for tag_frame, pose in snapshot.relocated.items():
            belief = self.belief.get(tag_frame)
            if belief is not None:
                self.belief[tag_frame] = (list(pose), belief[1])
            self.get_logger().info(f"{tag_frame}: carried with a slid stack")
        for tag_frame in snapshot.lost:
            why = snapshot.lost_why.get(tag_frame, "no reason recorded")
            self.get_logger().warn(f"{tag_frame}: lost: {why}")
            self.belief.pop(tag_frame, None)
            for source in SOURCES:
                self.forget((tag_frame, source))
            self.state[tag_frame] = None
            poses.pop(tag_frame, None)

        # A cube's height is known exactly from the level it is on (the cubes are 50 mm
        # and the table is where the CAD says), so the arm is given that and not the
        # noisy, calibration-dependent measurement. x and y stay as measured.
        exact = exact_heights(snapshot.columns)
        self.exact_z = exact
        for tag_frame, pose in poses.items():
            if not should_publish(states.get(tag_frame), camera_alive):
                continue
            if tag_frame in exact and states.get(tag_frame) != "held":
                position, rotation = pose
                pose = ([position[0], position[1], exact[tag_frame]], rotation)
            self.publish(tag_frame, pose, now)
        self.ticks += 1
        if self.ticks % PUBLISH_EVERY == 0:
            self.publish_world(snapshot, now)


def main():
    rclpy.init()
    try:
        rclpy.spin(WorldState())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
