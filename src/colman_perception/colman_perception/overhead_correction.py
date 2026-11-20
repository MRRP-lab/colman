"""Estimate the height of a tag seen by the overhead camera, and correct its x and y.

The overhead camera gets a tag's depth from how big the tag looks, and a 39 mm tag is
only about 60 px wide at 4K from 1.8 m, so its height comes out low, by an amount that
changes from place to place on the table: for single cubes of the same true height the
raw reading spans about 20 mm. A least-squares fit to tags at exactly known heights
removes the average, and what is left (about 1.5 cm at worst) is why this height is only
used to decide which level a cube is on, never to tell the arm where the cube is: the
arm is given the exact height of the level (see stack_world.level_height). A wrong depth
also moves the tag along the camera's line of sight, so x and y are moved back along
that line to where it meets the corrected height.

The known heights are those of a cube top: the table surface is at z = -0.005 in
base_link (the touched table marks average -0.006 and the MoveIt table box has its top
at -0.005), and a cube is exactly 50 mm, so n cubes up is -0.005 + 0.05 n.
"""

# The overhead camera's position in base_link: the static_transform_args in
# config/overhead_camera_pose_in_base_link.yaml (a test keeps them in step).
CAMERA_XYZ = (0.33275, 0.05225, 1.82645)

# true z = Z_SLOPE * measured z + Z_OFFSET. Fitted at 4K with the 39.2 mm tag size and
# the camera pose in config/overhead_camera_pose_in_base_link.yaml, to 22 tags between 1
# and 8 cubes high (eight singles spread over the table, stacks of 2 and 3, one stack
# grown from 4 to 8): rms residual 6.4 mm, worst 15.6 mm. Position terms did no better
# under leave-one-out. Refit if the tag size, the camera pose or the focus changes.
Z_SLOPE = 1.0059
Z_OFFSET = 0.0148


def corrected_z(z):
    return Z_SLOPE * z + Z_OFFSET


def correct_position(position):
    """(x, y, z) of an overhead-camera tag in base_link, with all three corrected."""
    x, y, z = position
    cam_x, cam_y, cam_z = CAMERA_XYZ
    fixed = corrected_z(z)
    scale = (cam_z - fixed) / (cam_z - z)  # along the line from the camera to the tag
    return (cam_x + (x - cam_x) * scale, cam_y + (y - cam_y) * scale, fixed)
