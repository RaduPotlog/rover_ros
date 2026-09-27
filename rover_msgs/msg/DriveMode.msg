# Driving mode owned by rover_drive_mode (rover_orchestrator) (latched: transient-local, depth 1)

uint8 MANUAL = 1            # Web joystick straight to the platform, no obstacle check
uint8 ASSISTED = 2          # Web joystick through the lidar collision monitor (slow down, stop)
uint8 AUTOMATIC = 3         # Nav 2 drives (GoTo); moving the web joystick takes over to ASSISTED
# 0 is deliberately unused: a default-constructed message is not a valid mode

uint8 GUARD_BYPASSED = 0    # MANUAL: the teleop collision monitor is not in the path
uint8 GUARD_CLEAR = 1       # Nothing in the slow-down or stop zones
uint8 GUARD_SLOWING = 2     # Obstacle in the slow-down zone, speed scaled down
uint8 GUARD_STOPPED = 3     # Obstacle in the stop zone (or stale lidar), motion blocked
uint8 GUARD_NO_DATA = 4     # No recent collision monitor state

std_msgs/Header header
uint8 mode
uint8 guard                 # State of the collision monitor guarding the active mode
string reason               # Why the mode last changed ("boot default", "operator request", ...)
