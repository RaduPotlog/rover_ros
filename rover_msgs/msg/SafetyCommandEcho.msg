# Read-back of the safety-PLC coils that SOFTWARE drives.
#
# Every field here is an echo of something this system wrote. It confirms the PLC accepted a
# write; it says NOTHING about the state of the plant. The rule is therefore one-directional:
#
#   * an asserted field may be used to INHIBIT motion - "we asked for a stop" is a sound reason
#     not to drive, and it is visible a poll or two before the resulting latch is;
#   * no field here may be used to PERMIT motion, or read as evidence of what the hardware is
#     actually doing. Only SafetyStatus can say that.
#
# rover_twist_mux gates on sw_e_stop_user_button and sw_e_stop_motor_driver_fault under the first
# rule, alongside the plant state from SafetyStatus.
#
# Commands themselves are services, not a topic: hardware_interface/sw_user_e_stop_set,
# .../sw_user_e_stop_reset and .../sw_e_stop_latch_reset. A safety command needs an
# acknowledgement and must not be replayed to a late joiner, which a latched command topic would
# do - which is why there is no GpioControl message and should not be one.

std_msgs/Header header

# When the safety PLC was actually polled - see SafetyStatus.io_sample_time.
builtin_interfaces/Time io_sample_time

# Software E-Stop user button coil (COIL_2). true = we are asserting the stop.
bool sw_e_stop_user_button

# Motor-driver fault stop coil (COIL_3). true = we are asserting the stop.
bool sw_e_stop_motor_driver_fault

# Latch-reset coil (COIL_4). A write-only command pulse - driven true then immediately false - so
# this reads false essentially always. It is NOT a state; gating on it would invert the meaning
# of a reset. The observable latch state is SafetyStatus.latch_active.
bool sw_e_stop_latch_reset

# CPU watchdog heartbeat coil (COIL_1). A square wave toggled every safety_wdg_kick_period_ms to
# keep the relay's watchdog fed - a liveness heartbeat, never a fault flag. NEITHER LEVEL MEANS
# ANYTHING about whether motion is safe; gating on it once made a motion lock oscillate. A
# stalled heartbeat is caught by the relay, which latches the E-Stop, and that surfaces as
# SafetyStatus.latch_active.
bool cpu_wdg_heartbeat
