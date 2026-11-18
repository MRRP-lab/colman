"""
Chases an AprilTag cube with the wrist camera, like a third-person game camera: it
stays 20 cm from the cube, behind it (on the robot's side) and raised above it,
looking at it with the image level, and moves with the cube.

Reads the tag poses from TF, works out the camera twist with follow_geometry and sends
it to MoveIt Servo. It only moves the arm between /follow/start and /follow/stop.
With the `dry` parameter it does everything except send the twist, so the aim can be
checked first (watch /follow/goal_pose in RViz).

Every change in Servo's status, such as a joint limit, a collision or a singularity, is
logged with the joint positions and camera pose, to find limits the MoveIt model gets
wrong.
"""

import time

import numpy as np
import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from colman_motion.diff_ik import dls_joint_velocity
from colman_motion.follow_geometry import (
    ABOVE_CUBE_M,
    STANDOFF_M,
    axis_from_fourfold,
    chase_position,
    follow_twist,
    fourfold,
    goal_pose,
    horizontal_axis,
    lowpass,
    matrix_to_quat,
    quat_to_matrix,
)

BASE_FRAME = "base_link"
CAMERA_FRAME = "oakd_pro_camera_rgb_optical_frame_wrist"
# The tip of the ur_arm group: the frame Servo actually moves and turns.
SERVO_FRAME = "tool0"
# The arm's joints in the order MoveIt lists them for the ur_arm group.
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
WRIST_3_JOINT = "wrist_3_joint"
ELBOW_JOINT = "elbow_joint"
# The camera's motion is turned into joint velocities by damped least squares, which
# slows the arm in the one direction it can't move near a singularity instead of
# freezing, and never pushes a joint past its limit. This caps the fastest joint.
MAX_JOINT_RPS = 1.0
TAG_IDS = range(1, 9)
# Each camera names a tag's frame differently; the tag ID is the same in both.
WRIST_FRAME = "tag_{}"
OVERHEAD_FRAME = "ovh_tag_{}"
# The overhead camera runs at about 5 Hz, so its detections are allowed to be older.
# It is also less accurate, so the wrist camera wins when both see the cube.
OVERHEAD_MAX_AGE_S = 0.8
RATE_HZ = 25.0
# Tag poses older than this are not used; the arm holds still instead.
TAG_MAX_AGE_S = 0.3
# Smoothing of the cube position: 1.0 is none.
POSITION_ALPHA = 0.3
# A cube that drops out of detection for less than this is assumed to be where it
# was, so a flickering detection doesn't make the arm stop and start.
LOST_GRACE_S = 0.6
# Smoothing of the cube's yaw (a 4-fold angle, since a cube repeats every 90 degrees).
YAW_ALPHA = 0.2

# moveit_servo's StatusCode values (Jazzy). Anything else is logged by number.
SERVO_STATUS = {
    -1: "INVALID",
    0: "NO_WARNING",
    1: "DECELERATE_FOR_APPROACHING_SINGULARITY",
    2: "HALT_FOR_SINGULARITY",
    3: "DECELERATE_FOR_LEAVING_SINGULARITY",
    4: "DECELERATE_FOR_COLLISION",
    5: "HALT_FOR_COLLISION",
    6: "JOINT_BOUND",
}


class CubeFollower(Node):
    def __init__(self):
        super().__init__("cube_follower")
        self.declare_parameter("dry", False)
        self.declare_parameter("standoff", STANDOFF_M)
        # How far above the cube the camera hangs.
        self.declare_parameter("above", ABOVE_CUBE_M)
        self.above = self.get_parameter("above").value
        # 0 locks onto the first tag seen after /follow/start; set it to follow one tag.
        self.declare_parameter("tag_id", 0)
        self.fixed_tag_id = self.get_parameter("tag_id").value
        self.locked_id = self.fixed_tag_id or None
        self.dry = self.get_parameter("dry").value
        self.standoff = self.get_parameter("standoff").value

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.joint_pub = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, "/follow/goal_pose", 10)
        self.create_subscription(ServoStatus, "/servo_node/status", self.on_status, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.switch_client = self.create_client(
            ServoCommandType, "/servo_node/switch_command_type"
        )
        self.create_service(Trigger, "/follow/start", self.start_cb)
        self.create_service(Trigger, "/follow/stop", self.stop_cb)

        self.following = False
        self.cube = None
        self.joints = {}
        self.status = 0
        self.seen = False
        self.last_seen_s = 0.0
        self.source = None
        self.pending_switch = None
        self.cube_yaw = None
        self.create_timer(1.0 / RATE_HZ, self.tick)
        mode = "dry run, will not move the arm" if self.dry else "live"
        self.get_logger().info(f"Ready ({mode}); call /follow/start to begin")

    def start_cb(self, request, response):
        if self.following or self.pending_switch is not None:
            response.success = True
            response.message = "already following"
            return response
        if self.dry:
            self.following = True
            self.cube = None
            self.get_logger().info("Following started (dry)")
            response.success = True
            response.message = "following (dry)"
            return response
        if not self.switch_client.service_is_ready():
            response.success = False
            response.message = (
                "Servo is not running (no /servo_node/switch_command_type)"
            )
            return response
        # Servo ignores joint commands until it is switched to joint-jog mode. The
        # reply is checked in tick, so this callback doesn't block the executor.
        request_msg = ServoCommandType.Request()
        request_msg.command_type = ServoCommandType.Request.JOINT_JOG
        self.pending_switch = self.switch_client.call_async(request_msg)
        response.success = True
        response.message = "starting"
        return response

    def stop_cb(self, request, response):
        self.halt()
        response.success = True
        response.message = "stopped"
        return response

    def halt(self):
        self.following = False
        self.pending_switch = None
        self.source = None
        self.locked_id = self.fixed_tag_id or None
        self.cube = None
        self.cube_yaw = None
        self.publish_joint_velocities(np.zeros(6))
        self.get_logger().info("Following stopped")

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def on_status(self, msg):
        if msg.code == self.status:
            return
        name = SERVO_STATUS.get(msg.code, f"code {msg.code}")
        previous = SERVO_STATUS.get(self.status, f"code {self.status}")
        self.status = msg.code
        joints = ", ".join(
            f"{n}={p:+.3f}" for n, p in self.joints.items() if "gripper" not in n
        )
        self.get_logger().warn(f"Servo status {previous} -> {name}; joints: {joints}")

    def lookup(self, target, source, max_age_s=None):
        """Transform of `source` in `target`, or None if missing or too old."""
        try:
            found = self.buffer.lookup_transform(
                target, source, Time(), Duration(seconds=0.0)
            )
        except TransformException:
            return None
        if max_age_s is not None:
            age = (
                self.get_clock().now() - Time.from_msg(found.header.stamp)
            ).nanoseconds
            if age > max_age_s * 1e9:
                return None
        return found.transform

    def tag_pose(self, frame, max_age_s):
        """The live (position, rotation matrix) of a tag frame in the base frame."""
        transform = self.lookup(BASE_FRAME, frame, max_age_s)
        if transform is None:
            return None
        t, q = transform.translation, transform.rotation
        return np.array([t.x, t.y, t.z]), quat_to_matrix(q.x, q.y, q.z, q.w)

    def tag_position(self, frame, max_age_s):
        """The live position of a tag frame in the base frame, or None."""
        pose = self.tag_pose(frame, max_age_s)
        return None if pose is None else pose[0]

    def find_cube(self):
        """
        (position in the base frame, camera name, a horizontal direction along
        one of the cube's edges) of the cube, or (None, None, None).
        The first tag seen is locked on, so a second cube or another tag on the same
        scene can't pull the arm away. Either camera will do; the wrist camera wins.
        """
        ids = [self.locked_id] if self.locked_id else TAG_IDS
        best = None
        # Everything visible, for the log: which camera put which tag where.
        visible = []
        for camera, template, max_age_s in (
            ("wrist", WRIST_FRAME, TAG_MAX_AGE_S),
            ("overhead", OVERHEAD_FRAME, OVERHEAD_MAX_AGE_S),
        ):
            for tag_id in TAG_IDS:
                position = self.tag_position(template.format(tag_id), max_age_s)
                if position is not None:
                    x, y, z = position
                    visible.append(
                        f"{camera} tag {tag_id} ({x:+.3f}, {y:+.3f}, {z:+.3f})"
                    )
        self.get_logger().info(
            f"tags seen (base frame, locked on {self.locked_id}): "
            + ("; ".join(visible) or "none"),
            throttle_duration_sec=1.0,
        )
        for camera, template, max_age_s in (
            ("wrist", WRIST_FRAME, TAG_MAX_AGE_S),
            ("overhead", OVERHEAD_FRAME, OVERHEAD_MAX_AGE_S),
        ):
            for tag_id in ids:
                pose = self.tag_pose(template.format(tag_id), max_age_s)
                # A tag's frame stays in TF after it is lost, so the age check is
                # what tells a live detection from an old one.
                if pose is None:
                    continue
                position = pose[0]
                if best is None or np.linalg.norm(position) < np.linalg.norm(best[0]):
                    best = (position, camera, tag_id, pose[1])
            if best is not None:
                break
        if best is None:
            return None, None, None
        if self.locked_id is None and self.following:
            self.locked_id = best[2]
            self.get_logger().info(f"Locked onto tag {best[2]}")
        return best[0], best[1], horizontal_axis(best[3])

    def camera_pose(self):
        transform = self.lookup(BASE_FRAME, CAMERA_FRAME)
        if transform is None:
            return None
        t, q = transform.translation, transform.rotation
        rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
        return np.array([t.x, t.y, t.z]), rotation

    def joint_vector(self):
        """The arm's joint angles in JOINTS order, or None if any is missing."""
        try:
            return np.array([self.joints[name] for name in JOINTS])
        except KeyError:
            return None

    def publish_joint_velocities(self, velocities):
        if self.dry:
            return
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.joint_names = list(JOINTS)
        msg.velocities = [float(v) for v in velocities]
        self.joint_pub.publish(msg)

    def publish_goal(self, camera, cube, camera_x, cube_axis):
        goal = goal_pose(
            camera,
            cube,
            self.standoff,
            camera_x,
            above=self.above,
            cube_axis=cube_axis,
        )
        if goal is None:
            return
        position, rotation = goal
        x, y, z, w = matrix_to_quat(rotation)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(
            float, position
        )
        msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
        self.goal_pub.publish(msg)

    def tick(self):
        if self.pending_switch is not None and self.pending_switch.done():
            switch = self.pending_switch
            self.pending_switch = None
            if switch.exception() is None and switch.result().success:
                self.following = True
                self.cube = None
                self.get_logger().info("Servo is in joint-jog mode, following started")
            else:
                self.get_logger().error("Servo refused joint-jog mode, not following")
        camera = self.camera_pose()
        seen, source, axis = self.find_cube()
        now = time.monotonic()
        remembered = (
            seen is None
            and self.cube is not None
            and now - self.last_seen_s < LOST_GRACE_S
        )
        if camera is None or (seen is None and not remembered):
            if self.following and self.seen:
                self.get_logger().warn("Cube not visible, holding still")
            self.seen = False
            self.cube = None
            if self.following:
                self.publish_joint_velocities(np.zeros(6))
            return
        if seen is not None:
            if not self.seen:
                self.get_logger().info(f"Cube seen by the {source} camera")
            elif source != self.source:
                self.get_logger().debug(f"Cube now seen by the {source} camera")
            if source != self.source:
                # The two cameras disagree by a few cm, so don't blend across the
                # change.
                self.cube = None
                self.cube_yaw = None
            self.source = source
            self.seen = True
            self.last_seen_s = now
            self.cube = lowpass(self.cube, seen, POSITION_ALPHA)
            if axis is not None:
                self.cube_yaw = lowpass(self.cube_yaw, fourfold(axis), YAW_ALPHA)
        cube_axis = None if self.cube_yaw is None else axis_from_fourfold(self.cube_yaw)
        self.publish_goal(camera[0], self.cube, camera[1][:, 0], cube_axis)
        if not self.following:
            return
        linear, angular, distance_error, aim_error, roll_error = follow_twist(
            camera[0],
            camera[1],
            self.cube,
            self.standoff,
            above=self.above,
            cube_axis=cube_axis,
            wrist_roll=self.joints.get(WRIST_3_JOINT),
        )
        goal = chase_position(self.cube, self.standoff, above=self.above)
        self.get_logger().info(
            f"cube ({self.cube[0]:+.3f}, {self.cube[1]:+.3f}, {self.cube[2]:+.3f}) "
            f"from {self.source}; camera ({camera[0][0]:+.3f}, {camera[0][1]:+.3f}, "
            f"{camera[0][2]:+.3f}), looking along "
            f"({camera[1][0, 2]:+.2f}, {camera[1][1, 2]:+.2f}, "
            f"{camera[1][2, 2]:+.2f}); "
            f"goal ({goal[0]:+.3f}, {goal[1]:+.3f}, {goal[2]:+.3f}); "
            f"pos err {distance_error:.3f} m, aim {np.degrees(aim_error):.1f} deg, "
            f"wrist_3 {self.joints.get(WRIST_3_JOINT, float('nan')):+.2f} rad, "
            f"elbow {self.joints.get(ELBOW_JOINT, float('nan')):+.2f} rad",
            throttle_duration_sec=1.0,
        )
        if self.dry:
            self.get_logger().info(
                f"dry: dist err {distance_error:+.3f} m, "
                f"aim {np.degrees(aim_error):.1f} deg, "
                f"roll {np.degrees(roll_error):+.1f} deg, "
                f"linear {np.round(linear, 3)}, angular {np.round(angular, 3)}",
                throttle_duration_sec=0.5,
            )
        # Joint velocities that move the camera as asked (damped least squares: it
        # slows near singularities rather than freezing, and respects joint limits).
        q = self.joint_vector()
        tip = self.lookup(BASE_FRAME, SERVO_FRAME)
        if q is None or tip is None:
            self.publish_joint_velocities(np.zeros(6))
            return
        tip_rotation = quat_to_matrix(
            tip.rotation.x, tip.rotation.y, tip.rotation.z, tip.rotation.w
        )
        tip_position = np.array(
            [tip.translation.x, tip.translation.y, tip.translation.z]
        )
        camera_offset = tip_rotation.T @ (camera[0] - tip_position)
        velocities = dls_joint_velocity(
            q,
            np.concatenate([linear, angular]),
            camera_offset,
            max_joint_velocity=MAX_JOINT_RPS,
        )
        self.publish_joint_velocities(velocities)


def main():
    rclpy.init()
    node = CubeFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.halt()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
