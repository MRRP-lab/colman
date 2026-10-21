import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

BASE_FRAME = "base_link"

# Samples are only kept while every joint is moving slower than this.
MAX_JOINT_SPEED_RAD_S = 0.02

# Samples further than this many MADs from the median are dropped from the average.
OUTLIER_MADS = 3.0

LOG_PERIOD_S = 2.0


def average_quaternion(quaternions):
    """
    The average of unit quaternions (x, y, z, w), the dominant eigenvector
    of their outer products, so q and -q count as the same rotation.
    """
    q = np.asarray(quaternions)
    _, vectors = np.linalg.eigh(q.T @ q)
    mean = vectors[:, -1]
    return mean if mean[3] >= 0 else -mean


def trim_outliers(positions):
    median = np.median(positions, axis=0)
    distance = np.linalg.norm(positions - median, axis=1)
    mad = np.median(distance)
    if mad == 0.0:
        return np.ones(len(positions), bool)
    return distance <= OUTLIER_MADS * 1.4826 * mad


class TagRecorder(Node):
    def __init__(self):
        super().__init__("tag_recorder")
        self.declare_parameter("tag_frames", ["tag_1", "tag_3", "tag_5", "tag_6"])
        self.declare_parameter(
            "output",
            "src/colman_perception/colman_perception/config/tag_map.yaml",
        )
        self.tags = list(self.get_parameter("tag_frames").value)
        self.output = str(self.get_parameter("output").value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.samples = {tag: [] for tag in self.tags}
        self.last_stamp = {}
        self.still = False
        self.create_subscription(JointState, "/joint_states", self.joints_cb, 10)
        self.create_timer(0.05, self.sample)
        self.create_timer(LOG_PERIOD_S, self.report)
        self.get_logger().info(
            f"Recording {self.tags} in {BASE_FRAME}, hold the arm still to collect"
        )

    def joints_cb(self, msg):
        speed = max((abs(v) for v in msg.velocity), default=0.0)
        self.still = speed < MAX_JOINT_SPEED_RAD_S

    def sample(self):
        if not self.still:
            return
        for tag in self.tags:
            try:
                transform = self.buffer.lookup_transform(
                    BASE_FRAME, tag, rclpy.time.Time()
                )
            except TransformException:
                continue
            stamp = (
                transform.header.stamp.sec,
                transform.header.stamp.nanosec,
            )
            if self.last_stamp.get(tag) == stamp:
                continue
            self.last_stamp[tag] = stamp
            t = transform.transform.translation
            r = transform.transform.rotation
            self.samples[tag].append([t.x, t.y, t.z, r.x, r.y, r.z, r.w])

    def estimate(self, tag):
        data = np.array(self.samples[tag])
        if len(data) == 0:
            return None
        keep = trim_outliers(data[:, :3])
        data = data[keep]
        return {
            "position": data[:, :3].mean(axis=0),
            "std_mm": data[:, :3].std(axis=0) * 1000.0,
            "quaternion": average_quaternion(data[:, 3:]),
            "samples": int(len(data)),
        }

    def report(self):
        for tag in self.tags:
            e = self.estimate(tag)
            if e is None:
                self.get_logger().info(f"{tag}: not seen yet")
                continue
            x, y, z = e["position"]
            sx, sy, sz = e["std_mm"]
            self.get_logger().info(
                f"{tag}: n={e['samples']} "
                f"x={x:.4f} y={y:.4f} z={z:.4f} "
                f"std(mm)={sx:.2f}/{sy:.2f}/{sz:.2f}"
            )

    def save(self):
        result = {}
        for tag in self.tags:
            e = self.estimate(tag)
            if e is None:
                continue
            result[tag] = {
                "position": [float(v) for v in e["position"]],
                "quaternion": [float(v) for v in e["quaternion"]],
                "std_mm": [float(v) for v in e["std_mm"]],
                "samples": e["samples"],
            }
        with open(self.output, "w") as f:
            yaml.safe_dump({"frame": BASE_FRAME, "tags": result}, f)
        self.get_logger().info(f"Saved {len(result)} tags to {self.output}")


def main():
    rclpy.init()
    node = TagRecorder()
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
