"""Correct the height of a tag seen by the overhead camera, and its x and y with it.

The overhead camera gets a tag's depth from how big the tag looks, and a 39 mm tag is
only about 60 px wide at 4K from 1.8 m, so its height comes out a little low. The error
is repeatable. It was fitted by least squares against tags at known heights (single
cubes 0.05 m, a stack of two 0.10 m, a stack of three 0.15 m), and the fit is applied
here. A wrong depth also moves the tag along the camera's line of sight, so x and y are
moved back along that line to where it meets the corrected height.
"""

import copy

# The overhead camera's position in base_link, from config/overhead_camera.yaml.
CAMERA_XYZ = (0.34221, 0.04809, 1.83012)

# true z = Z_SLOPE * measured z + Z_OFFSET, fitted at 4K.
Z_SLOPE = 0.985
Z_OFFSET = 0.0124


def corrected_z(z):
    return Z_SLOPE * z + Z_OFFSET


def correct_overhead(transform):
    """A copy of an overhead-camera transform (in base_link) with x, y, z corrected."""
    out = copy.deepcopy(transform)
    t = out.translation
    cam_x, cam_y, cam_z = CAMERA_XYZ
    z = corrected_z(t.z)
    scale = (cam_z - z) / (cam_z - t.z)  # along the line from the camera to the tag
    t.x = cam_x + (t.x - cam_x) * scale
    t.y = cam_y + (t.y - cam_y) * scale
    t.z = z
    return out
