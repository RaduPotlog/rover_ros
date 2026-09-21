# Indoor localization mode managed by rover_indoor_nav_manager (latched: transient-local, depth 1)

uint8 UNAVAILABLE = 0       # No localization running
uint8 MAPPING = 1           # slam_toolbox is building a new map
uint8 LOCALIZATION = 2      # map_server + AMCL on a saved map
uint8 SWITCHING = 3         # Stopping one and starting the other

std_msgs/Header header
uint8 mode
string map_name             # Map in use (LOCALIZATION) or being built (MAPPING, may be empty)
string message
