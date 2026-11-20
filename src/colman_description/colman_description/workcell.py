"""The measured sizes of the cubes, their tags and the table, defined once.

Every package takes these from here, so that no one has to remember them. Using a wrong
one is how the arm once stopped a few centimetres above a cube: the cubes are exactly
50 mm, and 39.2 mm is the black square of the tag on top, not the cube.
"""

CUBE_SIZE_M = 0.05  # every cube is exactly 50 mm on each side
TAG_SIZE_M = 0.0392  # the black square of the AprilTag on a cube's top face
# The table surface in base_link: the MoveIt table box has its top here, and the arm's
# nozzle touched marks drawn on the table at -0.0045 to -0.0076 (about -0.006).
TABLE_TOP_Z = -0.005


def cube_top_z(level):
    """The exact height of the top of the cube at this level (1 is on the table)."""
    return TABLE_TOP_Z + level * CUBE_SIZE_M
