# Progress of the mission run by rover_mission_manager (latched: transient-local, depth 1)

uint8 IDLE = 0
uint8 RUNNING = 1
uint8 HELD = 2        # Suspended by the motion lock or a dead lidar; resumes on its own
uint8 SUCCEEDED = 3
uint8 FAILED = 4
uint8 CANCELLED = 5

std_msgs/Header header
string mission_id
uint8 state
uint32 current_index        # Waypoint being driven to (0-based)
uint32 total                # Number of waypoints in the mission
string message              # Failure reason, empty otherwise
