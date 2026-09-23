# General-purpose aux IO on the safety PLC's programmable digital I/O (Portenta Machine Control
# DIO00..DIO11), published by rover_hardware_interface on hardware_interface/aux_io_state.
#
# NOT part of the safety chain. Nothing here may gate motion or E-Stop logic - use SafetyStatus
# for that. Outputs are switched with the hardware_interface/aux_output_<i>/set services
# (std_srvs/SetBool).

std_msgs/Header header

# When the PLC was actually polled, as opposed to when this message was published. Time this
# out rather than trusting the message's arrival: the PLC itself only refreshes these pins every
# 100 ms, and the poll runs at the same rate.
builtin_interfaces/Time io_sample_time

# Input pins DIO06..DIO11 (Modbus coils 14..19). true = input active.
bool[6] inputs

# Read-back of output pins DIO00..DIO05 (Modbus coils 8..13): what the PLC is actually driving,
# not what was last requested. true = ON.
bool[6] outputs

# Both safety-controller threads alive and the last poll recent. When false, inputs/outputs are
# the last values successfully read and may be stale.
bool link_healthy
