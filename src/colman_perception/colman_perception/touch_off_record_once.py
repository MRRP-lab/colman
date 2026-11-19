"""
Records where the vacuum nozzle is, once, under a label. A non-interactive
version of touch_off_record: put the nozzle on a mark, then run

    touch_off_record_once <touched.yaml> <label> [--dry]

It averages 10 readings of the tip in base_link (the spread shows whether the
arm was still) and writes the point in the file touch_off_check reads. Touching
a label again replaces the earlier point. --dry prints without saving.

The tip offset is the vacuum nozzle's, from the URDF (vacuum_tip_link), in
tool0's axes. It was not measured on the real nozzle.
"""

import sys
import time

import numpy as np

from colman_perception.touch_off_record import (
    load,
    quaternion_to_matrix,
    save,
    tip_position,
)

TIP_OFFSET_M = [0.0, 0.0455, 0.088]
READINGS = 10
WAIT_S = 15.0


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    out, label = sys.argv[1], sys.argv[2]
    label = int(label) if label.isdigit() else label

    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node("touch_off_record_once")
    buffer = Buffer()
    TransformListener(buffer, node, spin_thread=True)

    def lookup():
        t = buffer.lookup_transform("base_link", "tool0", Time()).transform
        return tip_position(
            quaternion_to_matrix(
                t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
            ),
            [t.translation.x, t.translation.y, t.translation.z],
            TIP_OFFSET_M,
        )

    last, deadline = None, time.time() + WAIT_S
    while time.time() < deadline:
        try:
            lookup()
            break
        except Exception as e:  # tf2 has several exception types
            last = e
            time.sleep(0.2)
    else:
        sys.exit(f"no base_link -> tool0 transform: {last}")

    readings = []
    for _ in range(READINGS):
        readings.append(lookup())
        time.sleep(0.05)
    r = np.array(readings)
    xyz = r.mean(0)
    spread = np.ptp(r, axis=0) * 1000
    print(
        f"#{label}: tip x {xyz[0] * 1000:.1f} y {xyz[1] * 1000:.1f} "
        f"z {xyz[2] * 1000:.1f} mm"
        f"   (spread over readings {np.round(spread, 2).tolist()} mm)"
    )
    if "--dry" not in sys.argv:
        data = load(out)
        data["tip_offset_m"] = TIP_OFFSET_M
        data["points"] = [p for p in data.get("points", []) if p["label"] != label]
        data["points"].append(
            {"label": label, "xyz": [round(float(v), 5) for v in xyz]}
        )
        save(out, data)
        print(f"saved; {len(data['points'])} points in {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
