# Maps stored on the rover (latched: transient-local, depth 1)

std_msgs/Header header
rover_msgs/MapInfo[] maps
string active_map           # Map AMCL localizes on; empty while mapping or without a map
