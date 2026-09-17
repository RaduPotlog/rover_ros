# Raw CRSF RC channel values, decoded from a CRSF / ExpressLRS receiver.
#
# 11-bit unsigned counts straight off the wire, unscaled. Nominal endpoints are 172 (min),
# 992 (centre) and 1811 (max); a transmitter's own endpoints and trim move these, so tune
# rover_crsf_teleop's channel_in_* parameters against measured values, not against these.
#
# Index N-1 holds the channel the transmitter labels N: channels[2] is channel 3.
#
# stamp is taken when the frame was decoded, so `ros2 topic delay` measures real
# receiver-to-consumer latency. frame_id is empty: RC input has no TF frame.
std_msgs/Header header

uint16[16] channels
