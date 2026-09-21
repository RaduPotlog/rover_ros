# Progress of rover_crsf_teleop's RC calibration, published on rc/calibration/state.
#
# Reliable + transient-local, depth 1: a UI opening the page must see the current phase at once,
# unlike the rc/channels and rc/link echoes where a late sample is worthless.
std_msgs/Header header

# No calibration running. Teleop commands normally.
uint8 PHASE_IDLE=0
# Sampling the resting position of every channel - the operator is holding nothing.
uint8 PHASE_CENTER=1
# Tracking per-channel min/max - the operator is sweeping every stick and switch.
uint8 PHASE_SWEEP=2
# Measurement finished, waiting for the operator to apply or discard it.
uint8 PHASE_REVIEW=3
uint8 phase

# Frames accumulated in the current phase.
uint32 samples
# Progress through the current phase, 0.0-1.0. 0.0 when the phase has no defined end (the sweep
# runs until the operator says it is done).
float32 progress

# True while teleop is held off because a calibration is running. Always true when phase != IDLE.
bool teleop_inhibited

# The E-Stop state the node verified for itself from hardware_interface/gpio_state, which is what
# actually gates a calibration - the operator's confirmation is a second, independent condition.
# ENGAGED means any of the motion-inhibiting stops is active (hardware button, software button,
# motor-driver fault, or the safety latch), the same set rover_twist_mux locks motion on.
#
# UNKNOWN is not "probably fine": nothing has been received, or the last sample is too old to
# trust. A calibration is refused in that state, because the absence of evidence that the rover
# is safe to sweep is not evidence that it is.
uint8 ESTOP_UNKNOWN=0
uint8 ESTOP_ENGAGED=1
uint8 ESTOP_RELEASED=2
uint8 e_stop

# What the node is mapping sticks with right now.
rover_msgs/RcCalibration active
# The in-progress (or just-finished) measurement. Meaningless while phase is IDLE.
rover_msgs/RcCalibration measured

# Per channel: the sweep saw real travel, so `measured` carries a usable range for it. A channel
# that never moved keeps its `active` values when the measurement is applied.
bool[16] channel_moved

# Human-readable reasons a channel's measurement is unusable (collapsed half-throw, centre outside
# the measured range, ...). Empty when nothing is wrong.
string[] problems

# Human-readable summary of the last transition, for the operator.
string message
