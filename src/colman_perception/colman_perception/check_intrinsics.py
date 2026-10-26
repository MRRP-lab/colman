"""
Compares the intrinsics the OAK reports for the undistorted color image with
the factory calibration, scaled to the same size.

If they disagree by more than a pixel or so, the device-side undistortion is
not doing what we assume.

Usage: python check_intrinsics.py <view.npz> [oakd_pro_calibration.json]
"""

import json
import os
import sys

import numpy as np

# CAM_A, the color camera.
COLOR_SOCKET = 0

TOLERANCE_PX = 1.0

DEFAULT_CALIBRATION = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config", "oakd_pro_calibration.json"
)


def factory_intrinsics(path, width, height):
    """The factory color intrinsics, scaled from the calibrated size."""
    with open(path) as f:
        cameras = dict(json.load(f)["cameraData"])
    camera = cameras[COLOR_SOCKET]
    k = np.array(camera["intrinsicMatrix"], dtype=np.float64)
    sx = width / camera["width"]
    sy = height / camera["height"]
    # Pixel centers sit at half-integer coordinates, so the principal point
    # scales as (c + 0.5) * s - 0.5.
    k[0, 0] *= sx
    k[1, 1] *= sy
    k[0, 2] = (k[0, 2] + 0.5) * sx - 0.5
    k[1, 2] = (k[1, 2] + 0.5) * sy - 0.5
    return k


def compare(reported, factory):
    """Returns the differences (reported - factory) and whether all are small."""
    names = ["fx", "fy", "cx", "cy"]
    index = [(0, 0), (1, 1), (0, 2), (1, 2)]
    ok = True
    for name, i in zip(names, index):
        diff = reported[i] - factory[i]
        flag = "" if abs(diff) <= TOLERANCE_PX else "  <-- DISAGREES"
        ok = ok and not flag
        print(
            f"  {name}: reported {reported[i]:9.3f}  factory {factory[i]:9.3f}  "
            f"diff {diff:+7.3f} px{flag}"
        )
    return ok


def main():
    view_path = sys.argv[1]
    calibration = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_CALIBRATION
    data = np.load(view_path)
    reported = data["rgb_k"]
    height, width = data["rgb"].shape[:2] if "rgb" in data else (1080, 1920)
    factory = factory_intrinsics(calibration, width, height)
    print(f"Color intrinsics at {width}x{height}:")
    if compare(reported, factory):
        print("Agree within a pixel.")
    else:
        print("Disagree: the undistorted K is not the scaled factory K.")


if __name__ == "__main__":
    main()
