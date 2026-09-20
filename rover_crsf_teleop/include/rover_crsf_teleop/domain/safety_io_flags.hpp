// Copyright 2025 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROVER_CRSF_TELEOP_DOMAIN_SAFETY_IO_FLAGS_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_SAFETY_IO_FLAGS_HPP_

namespace rover_crsf_teleop
{

// Whether the rover's E-Stop is engaged, as far as this node can tell.
//
// kUnknown is a real answer, not a placeholder: nothing has arrived on
// hardware_interface/gpio_state, or the last sample is too old to trust. It is treated as
// "refuse", because the absence of evidence that the rover is safe to sweep is not evidence
// that it is.
enum class EStopState
{
    kUnknown,
    kEngaged,
    kReleased,
};

// The subset of rover_msgs/GpioState that decides whether the rover can be commanded to move,
// as plain bools so the rules stay free of ROS types.
//
// Polarity follows the hardware interface: every field is `true` when that stop is ACTIVE. There
// is no inversion anywhere in the chain - readDiscreteContact() returns the Modbus bit verbatim,
// updateGpioStateMsg() assigns it verbatim, and EmergencyStop::readEStopState() documents "the
// port reports `true` when the E-Stop is triggered ... no negation". rover_twist_mux's
// MotionLockPolicy inhibits motion on the same convention.
//
// NOTE ON THE DEFAULTS. rover_twist_mux's equivalent struct defaults every flag to `true`,
// because for it "assume a stop is active" denies motion and is therefore the safe guess. Here
// the meaning is reversed: an engaged E-Stop is what GRANTS permission to sweep the sticks, so
// defaulting to `true` would be a silent fail-open. These default to `false`, and a node that
// has heard nothing reports kUnknown rather than building a default-constructed value and
// calling it released.
struct SafetyIoFlags
{
    // Physical E-Stop button. `true` = pressed.
    bool hw_e_stop_user_button{false};

    // Software E-Stop user button, the one the RC switch and the safety node drive.
    // `true` = engaged.
    bool sw_e_stop_user_button{false};

    // Motor-driver fault stop. `true` = faulted.
    bool sw_e_stop_motor_driver_fault{false};

    // Safety latch. `true` = latched, a stop is being held until an explicit reset.
    bool sw_e_stop_latch_status{false};
};

// True when any one of the stops above is active, i.e. the rover cannot be driven and it is safe
// to sweep the sticks to full throw.
//
// This is the same set rover_twist_mux locks motion on, so "safe to calibrate" means exactly
// "twist_mux would refuse to move". Two fields of GpioState are deliberately NOT part of this
// struct, because including either would be a bug:
//
//   - gpio_pin_cpu_wdg_heartbeat is a ~1 Hz square wave the safety controller drives for
//     liveness, not a fault flag. It has already made a motion lock oscillate once.
//   - gpio_pin_sw_e_stop_latch_reset is a command pulse, written true then immediately false, so
//     it reads false essentially always. The observable state is sw_e_stop_latch_status.
bool motionIsInhibited(const SafetyIoFlags & flags);

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_SAFETY_IO_FLAGS_HPP_
