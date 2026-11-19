"""Named arm poses shared by the motion server, the demos and pick_and_place."""

import math

# The arm's joints in the order MoveIt lists them for the ur_arm group.
ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Where the arm goes to get out of the overhead camera's view of the table: shoulder
# straight up and forearm level, folded back over the base.
HOME_POSE = dict(zip(ARM_JOINTS, [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]))
