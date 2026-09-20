# Per-channel raw-count calibration for one CRSF / ExpressLRS transmitter.
#
# Index N-1 holds the channel the transmitter labels N, the same convention as RcChannels.msg:
# channel_mid[2] is channel 3's resting value.
#
# The values are raw 11-bit counts, unscaled, exactly as the decoder produces them. Nominal
# endpoints are 172 / 992 / 1811, but a transmitter's own endpoints, trim and expo move them -
# which is what rover_crsf_teleop's calibration measures.
std_msgs/Header header

# Lowest and highest count seen at full throw.
uint16[16] channel_min
# Resting (centred) count. The stick -> twist mapping is defined about this, so a centred stick
# produces exactly 0.0.
uint16[16] channel_mid
uint16[16] channel_max

# Half-width, in counts, of the band around channel_mid that maps to exactly 0.0. Sized against
# where the stick actually rests, not against frame noise.
uint16[16] channel_deadband
