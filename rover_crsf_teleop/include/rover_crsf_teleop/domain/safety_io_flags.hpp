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

// The plant state that decides whether it is safe to calibrate the RC sticks, as plain bools so
// the rules stay free of ROS types. Sourced from rover_msgs/SafetyStatus only.
//
// THIS IS A PERMIT, NOT AN INHIBIT. rover_twist_mux reads the same kind of signals the other way
// round: for it an active stop DENIES motion, so OR-ing in every stop it can find only ever stops
// more, and more inputs are safer. Here an active stop GRANTS permission to sweep the sticks to
// full throw while someone stands next to the rover. OR-ing more inputs into a permit only ever
// permits more, so the evidence is AND-ed instead, and only evidence that cannot be undone from
// somewhere else is accepted:
//
//   * hw_e_stop_user_button - the physical button. It is the only stop nothing can clear
//     remotely: the PLC latch is set-dominant, so while the button is down no
//     sw_e_stop_latch_reset call can re-energise the contactor. A latch set by the software
//     E-Stop (the RC switch, rover_safety, a service call) is NOT enough, because any of those
//     sources - or Foxglove, the Cockpit, a shell - can clear it with one Trigger call while the
//     operator is mid-sweep. This used to be OR-ed with the latch, which let exactly that
//     through: SW E-Stop on, latch set, physical button released, calibration allowed.
//   * sw_e_stop_latch_status - the PLC has actually acted on it.
//   * !motor_contactor_engaged - the drive is provably dead: the contacts are confirmed open by
//     the contactor's auxiliary contact, which is genuine feedback, not a mirror of the latch.
//
// The software stop *requests* (rover_msgs/SafetyCommandEcho) are deliberately absent: they are
// read-backs of coils this system writes, and granting a permit on our own request is a
// fail-open by construction.
//
// DEFAULTS are the non-permitting values - button released, latch clear, contactor engaged
// (drive assumed live). rover_twist_mux defaults the other way for the same reason in reverse.
// A node that has heard nothing reports kUnknown anyway rather than building one of these.
struct SafetyIoFlags
{
    // Physical E-Stop button, from the PLC's discrete input. `true` = pressed.
    bool hw_e_stop_user_button{false};

    // The PLC's own E-Stop latch. `true` = latched.
    bool sw_e_stop_latch_status{false};

    // Motor contactor aux-contact feedback. `true` = contacts CLOSED, drive live. Note the
    // inverted sense relative to the two fields above.
    bool motor_contactor_engaged{true};
};

// True only when the physical E-Stop is pressed, the PLC has latched, and the contactor has
// actually opened - i.e. the rover cannot be driven and nothing remote can change that. Named
// for what it decides rather than after rover_twist_mux's isMotionInhibited(): the two used to
// share a name while meaning opposite things, which is part of how the OR-ed version survived.
bool isSafeToCalibrate(const SafetyIoFlags & flags);

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_SAFETY_IO_FLAGS_HPP_
