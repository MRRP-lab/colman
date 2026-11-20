"""Correct the height of a tag seen by the overhead camera, and its x and y with it.

The overhead camera gets a tag's depth from how big the tag looks, and a 39 mm tag is
only about 60 px wide at 4K from 1.8 m, so its height comes out a little low, and a
little more the further the tag is from the camera's axis. The error is repeatable. It
was fitted by least squares against tags at exactly known heights, and the fit is
applied here. A wrong depth also moves the tag along the camera's line of sight, so x
and y are moved back along that line to where it meets the corrected height.

The known heights are those of a cube top: the table surface is at z = -0.005 in
base_link (the touched table marks average -0.006 and the MoveIt table box has its top
at -0.005), and a cube is exactly 50 mm, so n cubes up is -0.005 + 0.05 n.
"""

# The overhead camera's position in base_link: the static_transform_args in
# config/overhead_camera_pose_in_base_link.yaml (a test keeps them in step).
CAMERA_XYZ = (0.33275, 0.05225, 1.82645)

# true z = Z_SLOPE * measured z + Z_OFFSET + Z_RADIAL * r^2, r being how far (m) the tag
# is from the camera's axis. Fitted at 4K with the 39.2 mm tag size and the camera pose
# in config/overhead_camera_pose_in_base_link.yaml, to 14 tags between 1 and 8 cubes
# high at ten places on the table: rms residual 3.8 mm, worst 10 mm. Refit if the tag
# size, the camera pose or the camera focus changes.
Z_SLOPE = 0.9759
Z_OFFSET = 0.0249
Z_RADIAL = -0.0356


def corrected_z(z, off_axis_m2=0.0):
    return Z_SLOPE * z + Z_OFFSET + Z_RADIAL * off_axis_m2


def correct_position(position):
    """(x, y, z) of an overhead-camera tag in base_link, with all three corrected."""
    x, y, z = position
    cam_x, cam_y, cam_z = CAMERA_XYZ
    fixed = corrected_z(z, (x - cam_x) ** 2 + (y - cam_y) ** 2)
    scale = (cam_z - fixed) / (cam_z - z)  # along the line from the camera to the tag
    return (cam_x + (x - cam_x) * scale, cam_y + (y - cam_y) * scale, fixed)
