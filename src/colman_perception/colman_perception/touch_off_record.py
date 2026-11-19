"""
Records points the robot has touched, as ground truth for camera calibration.

Put a pointed tip on the tool flange and measure where its end is from the
tool0 origin, in tool0's axes. Jog the arm, by hand in freedrive or from the
pendant, until the tip touches a point, then press Enter here. The
tip's position in base_link comes from the arm's forward kinematics.

    touch_off_record <out.yaml> --tip X Y Z [--base base_link] [--tool tool0]

--tip is the offset in metres from the tool0 origin to the tip, expressed in
tool0's axes (a tip straight out of the flange is 0 0 L). Without it the
recorded point is the tool0 origin, which is only useful as a smoke test.

At the prompt, type the label of the point just touched: the mark's number
from touch_off_pick, or the corner's index from `touch_off_check --annotate`
if you are touching grid corners. Or:

    u   drop the last point
    q   save and quit

The file is rewritten after every point, and an existing file is extended, so
a session can be stopped and resumed. Its format is read by touch_off_check:

    grid: [7, 5]
    tip_offset_m: [0, 0, 0.1]
    points:
    - {label: 17, xyz: [0.31, 0.02, 0.0045]}
"""

import argparse
import os

import numpy as np
import yaml


def tip_position(rotation, translation, offset):
    """
    Where a tip sits in the base frame, given the tool0 pose (3x3 rotation and
    translation in the base frame) and the tip's offset in tool0's axes.
    """
    return np.asarray(translation, float) + np.asarray(rotation, float) @ np.asarray(
        offset, float
    )


def quaternion_to_matrix(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def load(path):
    if os.path.exists(path):
        return yaml.safe_load(open(path)) or {}
    return {}


def save(path, data):
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out")
    parser.add_argument("--tip", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    parser.add_argument("--base", default="base_link")
    parser.add_argument("--tool", default="tool0")
    parser.add_argument(
        "--grid",
        type=int,
        nargs=2,
        default=None,
        help="inner corners across and down, only when touching grid corners",
    )
    args = parser.parse_args()

    # Imported here so the helpers above can be used without ROS.
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node("touch_off_record")
    buffer = Buffer()
    TransformListener(buffer, node, spin_thread=True)

    data = load(args.out)
    if args.grid:
        data["grid"] = args.grid
    data["tip_offset_m"] = list(args.tip)
    data.setdefault("points", [])
    print(f"{len(data['points'])} points already in {args.out}")

    try:
        while True:
            answer = (
                input("label of the point touched (u undo, q quit): ").strip().lower()
            )
            if answer == "q":
                break
            if answer == "u":
                if data["points"]:
                    print(f"dropped {data['points'].pop()}")
                    save(args.out, data)
                continue
            if not answer:
                continue
            label = int(answer) if answer.isdigit() else answer
            try:
                tf = buffer.lookup_transform(args.base, args.tool, Time())
            except Exception as e:  # tf2 has several exception types
                print(f"no transform {args.base} -> {args.tool}: {e}")
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            xyz = tip_position(
                quaternion_to_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z], args.tip
            )
            data["points"].append(
                {"label": label, "xyz": [round(float(v), 5) for v in xyz]}
            )
            save(args.out, data)
            print(f"point {label}: {np.round(xyz * 1000, 1)} mm")
    finally:
        save(args.out, data)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
