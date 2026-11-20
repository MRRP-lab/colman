"""Correct the height of a tag seen by the overhead camera, and its x and y with it.

The overhead camera gets a tag's depth from how big the tag looks, and a 39 mm tag is
only about 60 px wide at 4K from 1.8 m, so its height comes out a little low. The error
is repeatable. It was fitted by least squares against tags at known heights (single
cubes 0.05 m, stacks of 2 to 8 cubes), and the fit is applied here. A wrong depth also
moves the tag along the camera's line of sight, so x and y are moved back along that
line to where it meets the corrected height.
"""

# The overhead camera's position in base_link: the static_transform_args in
# config/overhead_camera_pose_in_base_link.yaml (a test keeps them in step).
CAMERA_XYZ = (0.33275, 0.05225, 1.82645)

# true z = Z_SLOPE * measured z + Z_OFFSET, fitted at 4K with the 39.2 mm tag size and
# the camera pose in config/overhead_camera_pose_in_base_link.yaml. Ten tags from 5 to
# 40 cm (three singles at different places, stacks of 2 and 3, one stack grown from 4
# to 8): rms residual 3.0 mm, worst 5.6 mm. Refit if the tag size, the camera pose or
# the camera focus changes.
Z_SLOPE = 0.9781
Z_OFFSET = 0.0288


def corrected_z(z):
    return Z_SLOPE * z + Z_OFFSET


def correct_position(position):
    """(x, y, z) of an overhead-camera tag in base_link, with all three corrected."""
    x, y, z = position
    cam_x, cam_y, cam_z = CAMERA_XYZ
    fixed = corrected_z(z)
    scale = (cam_z - fixed) / (cam_z - z)  # along the line from the camera to the tag
    return (cam_x + (x - cam_x) * scale, cam_y + (y - cam_y) * scale, fixed)
