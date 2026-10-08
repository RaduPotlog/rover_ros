# The person a perception tracker follows from frame to frame (rover_perception, e.g.
# rover_perception_fmoc on the RealSense depth cloud). Consumers: rover_follow_me (orchestrator),
# which hands it to Nav 2's Following server.
#
# header.frame_id is the frame the position is in (rover_perception_fmoc: <ns>/odom);
# header.stamp is the time of the sensor data it was measured from.

uint8 NONE=0       # nobody tracked: nothing acquired yet, or reset
uint8 TRACKING=1   # measured in this frame
uint8 COASTING=2   # not seen this frame (occluded / out of view); position is a prediction
uint8 LOST=3       # not seen for too many frames; position is the last known one

std_msgs/Header header
uint8 state
geometry_msgs/Point position
geometry_msgs/Vector3 velocity   # m/s in header.frame_id
float32 confidence               # 0..1, tracker-specific
string source                    # tracker that produced it, e.g. "fmoc"
