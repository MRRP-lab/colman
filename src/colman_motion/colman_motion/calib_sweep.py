import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from colman_motion.arm_control import ArmControl
from colman_motion.click_move import level_camera_rotation, quaternion_to_matrix
from colman_motion.demo_camera import rotation_to_quaternion
from colman_motion.robot_ready import wait_for_robot
from colman_motion.scene_manager import SceneManager

BASE_FRAME = "base_link"
TOOL_FRAME = "tool0"
CAMERA_FRAME = "oakd_pro_camera_rgb_optical_frame_wrist"

# The tabletop in the base frame: x from NEAR to FAR, y from -HALF to +HALF.
TABLE_NEAR_X_M = 0.09
TABLE_FAR_X_M = 0.63
TABLE_HALF_WIDTH_M = 0.5625

# The camera sits at most this far from the base, which is about as far as
# the arm reaches with the camera held out in front of the hand.
MAX_CAMERA_RADIUS_M = 0.38
STANDOFF_M = 0.2

CAMERA_HEIGHTS_M = (0.22, 0.32)

# Points to look at, relative to a corner: toward the middle of the table, and
# along each of the two edges that meet there.
LOOK_INWARD_M = 0.15
LOOK_ALONG_M = 0.25

# A view this close to straight down has no natural "up" for the image, so the
# camera roll is chosen to keep the hand near the base instead.
STEEP_VIEW_Z = 0.9

# How long the arm sits still before data is taken, and how many depth frames
# are collected for the median.
SETTLE_S = 1.5
DEPTH_FRAMES = 10

# The arm must not creep more than this while the data is taken.
MAX_DRIFT_M = 0.0005

# The camera must be publishing before anything moves, and the latest color
# image must be no older than this when a view is taken.
STREAM_TIMEOUT_S = 20.0

# The camera can drop off USB and reconnect by itself. A view waits this long
# for it to come back, with the arm held still, before giving up.
STREAM_RECOVERY_S = 30.0
MAX_IMAGE_AGE_S = 1.0



class SweepError(Exception):
    """
    Something went wrong that makes the rest of the sweep pointless.
    """


def matrix(position, rotation):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = position
    return m


class Recorder(Node):
    """
    Keeps the latest color image, the recent depth images and camera info, and
    looks up frames.
    """

    def __init__(self):
        super().__init__("calib_recorder")
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.rgb = None
        self.rgb_time = 0.0
        self.rgb_k = None
        self.depth_k = None
        self.depths = []
        self.collecting = False
        self.create_subscription(
            Image, "/oakd_pro/rgb/image", self.rgb_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/oakd_pro/depth/image", self.depth_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "/oakd_pro/rgb/camera_info",
            self.rgb_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/oakd_pro/depth/camera_info",
            self.depth_info_cb,
            qos_profile_sensor_data,
        )

    def rgb_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            self.rgb = image
            self.rgb_time = time.time()

    def depth_cb(self, msg):
        if not self.collecting:
            return
        depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        with self.lock:
            self.depths.append(depth)

    def rgb_info_cb(self, msg):
        self.rgb_k = np.array(msg.k).reshape(3, 3)

    def depth_info_cb(self, msg):
        self.depth_k = np.array(msg.k).reshape(3, 3)

    def check_streams(self):
        """
        Raises SweepError unless the camera is publishing color, depth and both
        camera infos.
        """
        deadline = time.time() + STREAM_TIMEOUT_S
        while time.time() < deadline:
            if (
                self.rgb_k is not None
                and self.depth_k is not None
                and self.image_age() < MAX_IMAGE_AGE_S
            ):
                break
            time.sleep(0.1)
        missing = []
        if self.image_age() >= MAX_IMAGE_AGE_S:
            missing.append("/oakd_pro/rgb/image")
        if self.rgb_k is None:
            missing.append("/oakd_pro/rgb/camera_info")
        if self.depth_k is None:
            missing.append("/oakd_pro/depth/camera_info")
        with self.lock:
            self.depths = []
        self.collecting = True
        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self.lock:
                if len(self.depths) >= DEPTH_FRAMES:
                    break
            time.sleep(0.05)
        self.collecting = False
        with self.lock:
            count = len(self.depths)
        if count < DEPTH_FRAMES:
            missing.append(f"/oakd_pro/depth/image ({count}/{DEPTH_FRAMES} frames)")
        if missing:
            raise SweepError(
                "The camera is not publishing: "
                + ", ".join(missing)
                + ". Start it with `ros2 run colman_perception camera_publisher`."
            )

    def image_age(self):
        """
        Seconds since the last color image, or infinity if there has been none.
        """
        with self.lock:
            if self.rgb is None:
                return float("inf")
            return time.time() - self.rgb_time

    def pose(self, parent, child):
        """
        The 4x4 transform of child in parent, or None.
        """
        try:
            transform = self.buffer.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        t = transform.transform.translation
        r = transform.transform.rotation
        return matrix([t.x, t.y, t.z], quaternion_to_matrix((r.x, r.y, r.z, r.w)))

    def capture(self):
        """
        Collects one still view. Raises SweepError if the arm moved or data is missing.
        """
        if self.image_age() > MAX_IMAGE_AGE_S:
            self.get_logger().warn("The camera stopped publishing, waiting for it")
            deadline = time.time() + STREAM_RECOVERY_S
            while self.image_age() > MAX_IMAGE_AGE_S and time.time() < deadline:
                time.sleep(0.1)
        before = self.pose(BASE_FRAME, TOOL_FRAME)
        with self.lock:
            self.depths = []
        self.collecting = True
        deadline = time.time() + 10.0
        while time.time() < deadline:
            with self.lock:
                if len(self.depths) >= DEPTH_FRAMES:
                    break
            time.sleep(0.05)
        self.collecting = False
        after = self.pose(BASE_FRAME, TOOL_FRAME)
        with self.lock:
            depths = list(self.depths)
            rgb = None if self.rgb is None else self.rgb.copy()
        if self.image_age() > MAX_IMAGE_AGE_S:
            rgb = None
        camera_in_tool = self.pose(TOOL_FRAME, CAMERA_FRAME)
        missing = [
            name
            for name, value in (
                ("tool pose", before),
                ("tool pose after", after),
                ("camera in tool", camera_in_tool),
                ("a fresh color image", rgb),
                ("color info", self.rgb_k),
                ("depth info", self.depth_k),
            )
            if value is None
        ]
        if len(depths) < DEPTH_FRAMES:
            missing.append(f"depth frames ({len(depths)}/{DEPTH_FRAMES})")
        if missing:
            raise SweepError("Missing " + ", ".join(missing))
        drift = np.linalg.norm(after[:3, 3] - before[:3, 3])
        if drift > MAX_DRIFT_M:
            raise SweepError(f"The arm moved {drift * 1000:.2f} mm while capturing")
        stack = np.array(depths, dtype=np.float32)
        stack[stack == 0] = np.nan
        with np.errstate(all="ignore"):
            depth = np.nan_to_num(np.nanmedian(stack, axis=0)).astype(np.uint16)
        return {
            "rgb": rgb,
            "depth": depth,
            "base_from_tool": after,
            "tool_from_camera": camera_in_tool,
            "rgb_k": self.rgb_k,
            "depth_k": self.depth_k,
        }


def views():
    """
    The camera positions and the points on the table they look at, as
    (camera, target) pairs in the base frame.
    """
    corners = [
        (x, y)
        for x in (TABLE_NEAR_X_M, TABLE_FAR_X_M)
        for y in (-TABLE_HALF_WIDTH_M, TABLE_HALF_WIDTH_M)
    ]
    middle = np.array([(TABLE_NEAR_X_M + TABLE_FAR_X_M) / 2.0, 0.0])
    for corner in map(np.array, corners):
        outward = corner / np.linalg.norm(corner)
        radius = min(np.linalg.norm(corner) - STANDOFF_M, MAX_CAMERA_RADIUS_M)
        inward = (middle - corner) / np.linalg.norm(middle - corner)
        along_x = np.array([-np.sign(corner[0] - middle[0]), 0.0])
        along_y = np.array([0.0, -np.sign(corner[1])])
        for height in CAMERA_HEIGHTS_M:
            camera = np.array([*(outward * radius), height])
            for toward, distance in (
                (inward, LOOK_INWARD_M),
                (along_x, LOOK_ALONG_M),
                (along_y, LOOK_ALONG_M),
            ):
                yield camera, np.array([*(corner + distance * toward), 0.0])


def inward_camera_rotation(camera, direction):
    """
    Camera rotation (columns x right, y down, z forward) looking along direction
    with the image's down side toward the base axis. The camera is mounted to
    one side of the hand, so this puts the hand nearer the base than the camera
    rather than 10 cm farther out, where steep views are beyond the arm's reach.
    """
    y = -np.array([camera[0], camera[1], 0.0])
    y -= np.dot(y, direction) * direction
    y /= np.linalg.norm(y)
    return np.column_stack([np.cross(y, direction), y, direction])


def tool_goal(camera, target, camera_in_tool):
    """
    The tool position and quaternion that put the camera at `camera` looking at
    `target`.
    """
    direction = target - camera
    direction /= np.linalg.norm(direction)
    if abs(direction[2]) > STEEP_VIEW_Z:
        camera_rotation = inward_camera_rotation(camera, direction)
    else:
        camera_rotation = level_camera_rotation(direction)
    tool_rotation = camera_rotation @ camera_in_tool[:3, :3].T
    tool_position = camera - tool_rotation @ camera_in_tool[:3, 3]
    return tool_position, rotation_to_quaternion(*tool_rotation.T)


def solve_view(arm, position, quaternion, seed):
    """
    The arm state that puts the tool at the pose, on the IK branch nearest the
    seed state (the current state if None), and a reason string if there is no
    collision free solution. Nothing moves.
    """
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion
    state = arm.state_from_ik(pose, TOOL_FRAME, seed)
    if state is None:
        return None, "no IK solution"
    if not arm.is_valid(state):
        joints = ", ".join(
            f"{name} {value:.2f}" for name, value in state.joint_positions.items()
        )
        return None, f"the IK solution is in collision or out of bounds ({joints})"
    return state, None


def run():
    logger = rclpy.logging.get_logger("moveit_py.calib_sweep")
    # A new folder each run, unless CALIB_DIR names one to resume into.
    output = os.environ.get(
        "CALIB_DIR", time.strftime("calib_data_%Y%m%d_%H%M%S", time.localtime())
    )

    # The camera comes first: there is no point in starting the arm without it.
    recorder = Recorder()
    recorder_executor = SingleThreadedExecutor()
    recorder_executor.add_node(recorder)
    threading.Thread(target=recorder_executor.spin, daemon=True).start()
    logger.info("Checking the camera streams")
    recorder.check_streams()
    logger.info("The camera is publishing")

    wait_for_robot()
    ur = MoveItPy(node_name="moveit_py")

    arm = ArmControl(ur)
    scene = SceneManager(ur)

    executor = MultiThreadedExecutor()
    executor.add_node(arm)
    executor.add_node(scene)
    threading.Thread(target=executor.spin, daemon=True).start()

    params = PlanRequestParameters(ur, "ompl")
    params.max_velocity_scaling_factor = 0.3
    params.max_acceleration_scaling_factor = 0.3

    try:
        scene.add_box("table", "base_link", [2.0, 2.0, 0.01], (0.0, 0.0, -0.01))

        # Where the camera sits on the hand, which never changes.
        deadline = time.time() + STREAM_TIMEOUT_S
        camera_in_tool = None
        while time.time() < deadline:
            camera_in_tool = recorder.pose(TOOL_FRAME, CAMERA_FRAME)
            if camera_in_tool is not None and (
                recorder.pose(BASE_FRAME, TOOL_FRAME) is not None
            ):
                break
            camera_in_tool = None
            time.sleep(0.2)
        if camera_in_tool is None:
            raise SweepError(f"No transform between {TOOL_FRAME}, {CAMERA_FRAME}")

        goals = [
            tool_goal(camera, target, camera_in_tool) for camera, target in views()
        ][: int(os.environ.get("CALIB_VIEWS", "24"))]

        # Views already in the folder are kept, so a sweep that stopped (the
        # camera dropping off USB, say) resumes by running again with the same
        # CALIB_DIR.
        done = [
            index
            for index in range(len(goals))
            if os.path.exists(f"{output}/view_{index:02d}.npz")
            and os.path.exists(f"{output}/view_{index:02d}_rgb.png")
        ]
        if done:
            logger.info(f"Resuming, {len(done)} views already in {output}: {done}")

        # Solve every remaining view before anything moves, each from the one
        # before so the arm stays on one branch.
        logger.info(f"Solving {len(goals) - len(done)} views")
        solutions = {}
        failed = []
        seed = None
        for index, (position, quaternion) in enumerate(goals):
            if index in done:
                continue
            state, reason = solve_view(arm, position, quaternion, seed)
            if state is None:
                failed.append(f"view {index}: {reason}")
            else:
                seed = state
                solutions[index] = state
        if failed:
            raise SweepError("nothing was moved. " + "; ".join(failed))
        logger.info("All views solved")

        os.makedirs(output, exist_ok=True)
        for index, state in solutions.items():
            if not arm.move_to_state(state, params):
                raise SweepError(f"View {index}: no plan or the move failed")
            time.sleep(SETTLE_S)
            try:
                data = recorder.capture()
            except SweepError as error:
                raise SweepError(f"View {index}: {error}") from error
            # The .npz goes last: a view only counts as saved once it exists.
            cv2.imwrite(f"{output}/view_{index:02d}_rgb.png", data.pop("rgb"))
            np.savez(f"{output}/view_{index:02d}.npz", **data)
            logger.info(f"View {index}: saved ({index + 1}/{len(goals)})")
        logger.info(f"SWEEP COMPLETE: {len(goals)} views in {output}")
    finally:
        scene.clear_scene()
        executor.shutdown()
        recorder_executor.shutdown()


def main():
    rclpy.init()
    try:
        run()
    except SweepError as error:
        rclpy.logging.get_logger("moveit_py.calib_sweep").error(f"SWEEP FAILED: {error}")
        raise SystemExit(1)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
