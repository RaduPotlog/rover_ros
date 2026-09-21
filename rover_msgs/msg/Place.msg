# A named pose on a map ("endpoint" in Clearpath IndoorNav terms)

string id                   # Stable id, assigned by rover_indoor_nav_manager
string name                 # Operator-facing name
string map_name             # Map the pose belongs to
float64 x                   # m, in that map's frame
float64 y                   # m
float64 theta               # rad, [-pi, pi]
