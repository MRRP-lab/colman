import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

BASE_FRAME = "base_link"

# How many depth frames to take the per pixel median of.
FRAMES = 10

# The camera must not move more than this while the frames are collected.
MAX_DRIFT_M = 0.002
MAX_DRIFT_ROT = 0.005

# The arm must move this far from the last solved view before the next one.
NEW_VIEW_M = 0.01
NEW_VIEW_ROT = 0.05

MIN_DEPTH_M = 0.2
MAX_DEPTH_M = 3.0

# The tabletop is roughly level and near the base's height.
MAX_TILT_RAD = np.radians(15.0)
TABLE_Z_RANGE_M = (-0.15, 0.08)

RANSAC_ITERATIONS = 400
RANSAC_TOLERANCE_M = 0.006
REFINE_TOLERANCE_M = 0.004

# An edge pixel has something at least this much farther than the table within
# this many pixels beyond it.
FAR_M = 0.10
EDGE_REACH_PX = 6

EDGE_TOLERANCE_M = 0.004
LINE_ITERATIONS = 300
MAX_EDGES = 4
MIN_EDGE_LENGTH_M = 0.08

# Contour points this close to the image border are where the view ends, not
# where the table does.
BORDER_PX = 6

MIN_EDGE_POINTS = 30


def quaternion_to_matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pixel_rays(k, shape):
    """
    A unit-depth ray through every pixel in camera coordinates, (h, w, 3).
    """
    h, w = shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    return np.dstack(
        [(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], np.ones((h, w))]
    )


def fit_plane(points):
    """
    Least squares plane through points, as a unit normal (pointing up) and offset.
    """
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid)
    normal = vt[2] if vt[2][2] >= 0 else -vt[2]
    return normal, float(normal @ centroid)


def find_plane(points, rng):
    """
    RANSAC for the largest roughly level plane near the base's height.
    """
    best = None
    best_count = 0
    for _ in range(RANSAC_ITERATIONS):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        if normal[2] < 0:
            normal = -normal
        if np.arccos(min(1.0, normal[2])) > MAX_TILT_RAD:
            continue
        offset = float(normal @ sample[0])
        height = offset / normal[2]
        if not TABLE_Z_RANGE_M[0] <= height <= TABLE_Z_RANGE_M[1]:
            continue
        count = int(np.sum(np.abs(points @ normal - offset) < RANSAC_TOLERANCE_M))
        if count > best_count:
            best, best_count = (normal, offset), count
    if best is None:
        return None
    normal, offset = best
    for _ in range(3):
        inliers = np.abs(points @ normal - offset) < REFINE_TOLERANCE_M
        normal, offset = fit_plane(points[inliers])
    return normal, offset


def table_mask(depth_m, rays, rotation, translation, plane):
    """
    Pixels whose 3D point is on the plane, with the holes (cubes, dropouts) filled.
    """
    normal, offset = plane
    points = (rays * depth_m[..., None]) @ rotation.T + translation
    on_plane = (np.abs(points @ normal - offset) < REFINE_TOLERANCE_M) & (depth_m > 0)
    mask = on_plane.astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return None
    mask = (labels == 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])).astype(np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    filled = np.zeros_like(mask)
    cv2.drawContours(filled, contours, -1, 1, -1)
    return filled


def ray_on_plane(rays, pixels, rotation, translation, plane):
    """
    Where the rays through (u, v) pixels meet the plane, in the base frame.
    """
    normal, offset = plane
    direction = rays[pixels[:, 1], pixels[:, 0]] @ rotation.T
    scale = (offset - normal @ translation) / (direction @ normal)
    return translation + direction * scale[:, None]


def edge_pixels(mask, depth_m, rays, rotation, translation, plane):
    """
    Pixels on the border of the table region with something much farther away
    just beyond them. That's the table's real edge, not a dropout in the depth.
    """
    normal, offset = plane
    along = (rays @ rotation.T) @ normal
    with np.errstate(all="ignore"):
        plane_depth = (offset - normal @ translation) / along
    far = (depth_m > 0) & (plane_depth > 0) & (depth_m > plane_depth + FAR_M)
    reach = 2 * EDGE_REACH_PX + 1
    near_far = cv2.dilate(far.astype(np.uint8), np.ones((reach, reach), np.uint8)) > 0
    border = (mask > 0) & (cv2.erode(mask, np.ones((3, 3), np.uint8)) == 0)
    keep = border & near_far
    keep[:BORDER_PX] = keep[-BORDER_PX:] = False
    keep[:, :BORDER_PX] = keep[:, -BORDER_PX:] = False
    v, u = np.nonzero(keep)
    return np.column_stack([u, v])


def fit_edges(mask, depth_m, rays, rotation, translation, plane):
    """
    Straight edges of the table, as lines in the base frame's xy plane, found by
    repeated RANSAC over the edge pixels.
    """
    pixels = edge_pixels(mask, depth_m, rays, rotation, translation, plane)
    if len(pixels) < MIN_EDGE_POINTS:
        return []
    xy = ray_on_plane(rays, pixels, rotation, translation, plane)[:, :2]
    rng = np.random.default_rng(0)
    edges = []
    remaining = np.arange(len(xy))
    for _ in range(MAX_EDGES):
        if len(remaining) < MIN_EDGE_POINTS:
            break
        points = xy[remaining]
        best = None
        for _ in range(LINE_ITERATIONS):
            a, b = points[rng.choice(len(points), 2, replace=False)]
            direction = b - a
            length = np.linalg.norm(direction)
            if length < 0.02:
                continue
            direction /= length
            distance = np.abs((points - a) @ np.array([-direction[1], direction[0]]))
            count = int(np.sum(distance < EDGE_TOLERANCE_M))
            if best is None or count > best[0]:
                best = (count, a, direction)
        if best is None:
            break
        _, a, direction = best
        for _ in range(2):
            distance = np.abs((points - a) @ np.array([-direction[1], direction[0]]))
            inliers = distance < EDGE_TOLERANCE_M
            if inliers.sum() < 2:
                break
            dx, dy, x0, y0 = cv2.fitLine(
                points[inliers].astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01
            ).flatten()
            a, direction = np.array([x0, y0]), np.array([dx, dy])
        normal = np.array([-direction[1], direction[0]])
        inliers = np.abs((points - a) @ normal) < EDGE_TOLERANCE_M
        span = np.ptp((points[inliers] - a) @ direction) if inliers.any() else 0.0
        if inliers.sum() < MIN_EDGE_POINTS or span < MIN_EDGE_LENGTH_M:
            break
        residual = (points[inliers] - a) @ normal
        edges.append(
            {
                "point": [float(a[0]), float(a[1])],
                "direction": [float(direction[0]), float(direction[1])],
                "angle_deg": float(
                    np.degrees(np.arctan2(direction[1], direction[0])) % 180.0
                ),
                "rms_mm": float(np.sqrt(np.mean(residual**2)) * 1000.0),
                "points": int(inliers.sum()),
                "length_m": float(span),
                "pixels": pixels[remaining[inliers]],
                "line": (a, direction),
            }
        )
        remaining = remaining[~inliers]
    return edges


def corners(edges):
    """
    Where pairs of roughly perpendicular edges meet.
    """
    found = []
    for i in range(len(edges)):
        for j in range(i + 1, len(edges)):
            (p, d), (q, e) = edges[i]["line"], edges[j]["line"]
            if abs(np.dot(d, e)) > np.cos(np.radians(70.0)):
                continue
            s = np.linalg.solve(np.column_stack([d, -e]), q - p)
            found.append((p + s[0] * d).tolist())
    return found


def fit_table(depth_m, k, rotation, translation, seed=0):
    """
    Finds the tabletop plane and its straight edges in one depth image.
    Returns a dict, with the mask and edge pixels for drawing, or None.
    """
    rays = pixel_rays(k, depth_m.shape)
    valid = (depth_m > MIN_DEPTH_M) & (depth_m < MAX_DEPTH_M)
    points = ((rays * depth_m[..., None]) @ rotation.T + translation)[valid][::3]
    if len(points) < 1000:
        return None
    plane = find_plane(points, np.random.default_rng(seed))
    if plane is None:
        return None
    mask = table_mask(depth_m, rays, rotation, translation, plane)
    if mask is None:
        return None
    normal, offset = plane
    inliers = points[np.abs(points @ normal - offset) < REFINE_TOLERANCE_M]
    edges = fit_edges(mask, depth_m, rays, rotation, translation, plane)
    return {
        "height_m": float(offset / normal[2]),
        "tilt_deg": float(np.degrees(np.arccos(min(1.0, normal[2])))),
        "normal": normal.tolist(),
        "plane_rms_mm": float(
            np.sqrt(np.mean((inliers @ normal - offset) ** 2)) * 1000.0
        ),
        "edges": edges,
        "corners": corners(edges),
        "mask": mask,
    }


def draw_overlay(depth_m, result):
    image = cv2.applyColorMap(
        np.clip(depth_m / 3.0 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO
    )
    image[depth_m == 0] = 0
    tint = np.zeros_like(image)
    tint[result["mask"] > 0] = (255, 255, 255)
    image = cv2.addWeighted(image, 0.6, tint, 0.4, 0)
    for edge in result["edges"]:
        for u, v in edge["pixels"]:
            cv2.circle(image, (int(u), int(v)), 1, (0, 0, 255), -1)
    return cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)


class TableFinder(Node):
    """
    Fits the tabletop every time the arm settles somewhere new, and saves every
    view when it exits.
    """

    def __init__(self):
        super().__init__("table_finder")
        self.declare_parameter(
            "output", "src/colman_perception/colman_perception/config/table_map.yaml"
        )
        self.declare_parameter("debug_prefix", "")
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.k = None
        self.frame_id = None
        self.frames = []
        self.first_pose = None
        self.last_solved = None
        self.views = []
        self.create_subscription(
            CameraInfo,
            "/oakd_pro/depth/camera_info",
            self.info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, "/oakd_pro/depth/image", self.image_cb, qos_profile_sensor_data
        )

    def info_cb(self, msg):
        self.k = np.array(msg.k).reshape(3, 3)
        self.frame_id = msg.header.frame_id

    def camera_pose(self):
        try:
            transform = self.buffer.lookup_transform(BASE_FRAME, self.frame_id, Time())
        except TransformException:
            return None
        t = transform.transform.translation
        r = transform.transform.rotation
        return quaternion_to_matrix((r.x, r.y, r.z, r.w)), np.array([t.x, t.y, t.z])

    def image_cb(self, msg):
        if self.k is None:
            return
        pose = self.camera_pose()
        if pose is None:
            return
        # Wait for the arm to leave the last place it was solved.
        if (
            self.last_solved is not None
            and np.linalg.norm(pose[1] - self.last_solved[1]) < NEW_VIEW_M
            and np.linalg.norm(pose[0] - self.last_solved[0]) < NEW_VIEW_ROT
        ):
            return
        if self.first_pose is None or (
            np.linalg.norm(pose[1] - self.first_pose[1]) > MAX_DRIFT_M
            or np.linalg.norm(pose[0] - self.first_pose[0]) > MAX_DRIFT_ROT
        ):
            self.frames = []
            self.first_pose = pose
        self.frames.append(self.bridge.imgmsg_to_cv2(msg, "passthrough"))
        if len(self.frames) < FRAMES:
            return
        self.solve(*pose)
        self.last_solved = pose
        self.frames = []
        self.first_pose = None

    def solve(self, rotation, translation):
        stack = np.array(self.frames, dtype=np.float32)
        stack[stack == 0] = np.nan
        with np.errstate(all="ignore"):
            depth_m = np.nan_to_num(np.nanmedian(stack, axis=0)) / 1000.0
        result = fit_table(depth_m, self.k, rotation, translation)
        if result is None:
            self.get_logger().warn("No tabletop in this view")
            return
        index = len(self.views)
        self.get_logger().info(
            f"View {index}: tabletop z={result['height_m'] * 1000:.1f} mm, "
            f"tilt {result['tilt_deg']:.2f} deg, rms {result['plane_rms_mm']:.2f} mm"
        )
        for edge in result["edges"]:
            self.get_logger().info(
                f"  edge {edge['angle_deg']:.2f} deg through "
                f"({edge['point'][0]:.4f}, {edge['point'][1]:.4f}), "
                f"{edge['points']} points over {edge['length_m'] * 1000:.0f} mm, "
                f"rms {edge['rms_mm']:.2f} mm"
            )
        for x, y in result["corners"]:
            self.get_logger().info(f"  corner at x={x:.4f} y={y:.4f}")
        prefix = str(self.get_parameter("debug_prefix").value)
        if prefix:
            cv2.imwrite(f"{prefix}{index}.png", draw_overlay(depth_m, result))
        self.views.append(
            {
                "camera_position": [float(v) for v in translation],
                "height_m": result["height_m"],
                "tilt_deg": result["tilt_deg"],
                "plane_rms_mm": result["plane_rms_mm"],
                "edges": [
                    {
                        k: edge[k]
                        for k in (
                            "point",
                            "direction",
                            "angle_deg",
                            "rms_mm",
                            "points",
                            "length_m",
                        )
                    }
                    for edge in result["edges"]
                ],
                "corners": result["corners"],
            }
        )

    def save(self):
        output = str(self.get_parameter("output").value)
        with open(output, "w") as f:
            yaml.safe_dump({"frame": BASE_FRAME, "views": self.views}, f)
        self.get_logger().info(f"Saved {len(self.views)} views to {output}")


def main():
    rclpy.init()
    node = TableFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
