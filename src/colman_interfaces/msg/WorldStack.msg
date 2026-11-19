# One column of cubes on the table, listed bottom to top. A cube the camera cannot
# identify (covered before we saw it go on, or moved while hidden) is named "unknown".
string[] tags
geometry_msgs/Point position
uint8 height
# False when any cube in the column is "unknown" or the order was not watched.
bool order_known
