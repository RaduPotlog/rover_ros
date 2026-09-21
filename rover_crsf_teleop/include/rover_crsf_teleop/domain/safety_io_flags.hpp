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
// hardware_interface/safety_status, or the last sample is too old to trust. It is treated as
// "refuse", because the absence of evidence that the rover is safe to sweep is not evidence
// that it is.
enum class EStopState
{
    kUnknown,
    kEngaged,
    kReleased,
};

// The PLANT state that decides whether the rover can actually move, as plain bools so the rules
// stay free of ROS types. Sourced from rover_msgs/SafetyStatus only.
//
// Polarity: every field is `true` when that stop is ACTIVE. There is no inversion anywhere in the
// chain - readDiscreteContact() returns the Modbus bit verbatim, the publisher assigns it
// verbatim, and EmergencyStop::readEStopState() documents "the port reports `true` when the
// E-Stop is triggered ... no negation".
//
// WHY THE TWO sw_* STOPS ARE NOT HERE. They used to be, back when everything arrived on one
// undifferentiated GpioState topic. They are read-backs of coils this system writes - "we asked
// the PLC to trip" - and they now live in rover_msgs/SafetyCommandEcho. That distinction matters
// specifically here, because of which way this node reasons: an active stop is what GRANTS
// permission to sweep the sticks to full throw. Granting that on the strength of a request we
// issued ourselves would be a fail-open if the PLC never acted on it. `latch_active` is the
// PLC's own answer and follows a software stop request within one poll, so nothing is lost by
// waiting for it.
//
// (rover_twist_mux does still gate on those echoes - but it only ever uses them to INHIBIT
// motion, which is the safe direction. See rover_msgs/SafetyCommandEcho.)
//
// NOTE ON THE DEFAULTS. rover_twist_mux's equivalent struct defaults every flag to `true`,
// because for it "assume a stop is active" denies motion and is therefore the safe guess. Here
// the meaning is reversed, so defaulting to `true` would be a silent fail-open. These default to
// `false`, and a node that has heard nothing reports kUnknown rather than building a
// default-constructed value and calling it engaged.
struct SafetyIoFlags
{
    // Physical E-Stop button, from the PLC's discrete input. `true` = pressed.
    bool hw_e_stop_user_button{false};

    // The PLC's own E-Stop latch. `true` = latched, a stop is being held until an explicit reset.
    // This is the authoritative "the PLC has tripped" signal.
    bool sw_e_stop_latch_status{false};
};

// True when a stop is confirmed by the PLC itself, i.e. the rover cannot be driven and it is safe
// to sweep the sticks to full throw.
bool motionIsInhibited(const SafetyIoFlags & flags);

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_SAFETY_IO_FLAGS_HPP_
