# What the cameras say is on the table right now. A single cube is a column of one.
builtin_interfaces/Time stamp
WorldStack[] stacks
# The cube the arm is holding, or "".
string held
# Cubes we have lost track of: not seen, not covered, and not hidden by the arm.
string[] missing
# True while the overhead camera is not delivering images; nothing above is current.
bool stale
