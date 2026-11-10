"""Named arm poses shared by the motion server, the demos and pick_and_place."""

# The camera poses used to look for cubes, tried in this order. The arm stands upright
# over the base and only the wrists tilt the camera. Each pose frames one side of the
# table: the top of the frame sits on the table's far edge, and the bottom corner on its
# right (or left) edge. Defined in colman_moveit_config/srdf/arm.srdf.xacro.
SCAN_POSES = ["scan_for_cubes_right", "scan_for_cubes_left"]
