# Plant state of the rover's safety chain, as read back from the safety PLC.
#
# This is the half of the old GpioState that describes what the HARDWARE is doing, as opposed to
# what software asked it to do (that is SafetyCommandEcho). The split exists because GpioState
# flattened both into seven identical, unlabelled bools, so a consumer could not tell a genuine
# plant reading from an echo of a command it had itself issued - and two packages ended up
# hand-writing prose comments about which fields were safe to gate on.
#
# THIS is the message to gate on. Nothing in SafetyCommandEcho ever should be.

std_msgs/Header header

# When the safety PLC was actually polled, as opposed to when this message was published. The
# two differ by up to one poll period, and GpioState carried no timestamp at all - consumers
# stamped on arrival, so their staleness checks measured transport age rather than data age.
builtin_interfaces/Time io_sample_time

# Physical E-Stop button, read from the PLC's discrete input (CONTACT_0). true = pressed.
bool hw_e_stop_user_button

# Motor contactor auxiliary-contact feedback (COIL_0). true = contacts CLOSED, drive live.
#
# Note the inverted sense relative to every other field here: true is the permissive state. It is
# genuine feedback from the contactor itself, not a mirror of the latch, which is what makes it
# possible to detect a contactor that failed to open - see latch_active.
bool motor_contactor_engaged

# The PLC's E-Stop latch (COIL_5). true = latched; a stop is held until an explicit reset.
#
# latch_active && motor_contactor_engaged, sustained past the contactor's drop-out time, means
# the stop was commanded and the contacts did not open - suspect welded contacts. The hardware
# interface cross-checks exactly this and inhibits motion on it.
bool latch_active

# Which of the latch's trip sources fired, once the PLC can report it.
#
# The latch has four independent SET sources (SW user button, SW motor-driver fault, CPU watchdog
# timeout, HW E-Stop button) but exposes only the single latch_active bit, so a rover that stops
# cannot currently say why. Until the PLC program latches the cause into readable coils this is
# always LATCH_CAUSE_UNKNOWN; the field exists now so consumers and log formats do not have to
# change again later.
uint8 LATCH_CAUSE_UNKNOWN = 0
uint8 LATCH_CAUSE_SW_USER_BUTTON = 1
uint8 LATCH_CAUSE_SW_MOTOR_DRIVER_FAULT = 2
uint8 LATCH_CAUSE_CPU_WATCHDOG = 3
uint8 LATCH_CAUSE_HW_USER_BUTTON = 4
uint8 latch_cause

# Whether the link to the safety PLC and the threads servicing it are healthy. false means the
# values above are last-known-good rather than current, and must be treated as untrustworthy.
bool link_healthy
