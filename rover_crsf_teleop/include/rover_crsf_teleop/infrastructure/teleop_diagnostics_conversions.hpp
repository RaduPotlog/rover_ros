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


#ifndef ROVER_CRSF_TELEOP_INFRASTRUCTURE_TELEOP_DIAGNOSTICS_CONVERSIONS_HPP_
#define ROVER_CRSF_TELEOP_INFRASTRUCTURE_TELEOP_DIAGNOSTICS_CONVERSIONS_HPP_

#include <vector>

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include "rover_crsf_teleop/application/teleop_use_case.hpp"
#include "rover_crsf_teleop/domain/teleop_health.hpp"
#include "rover_crsf_teleop/infrastructure/ros2_trigger_safety_switch.hpp"

namespace rover_crsf_teleop
{

// Maps a HealthLevel onto a diagnostic_msgs/DiagnosticStatus level byte.
unsigned char toDiagnosticLevel(HealthLevel level);

// "RC link" task. While teleop is inactive it is deliberately off the command path, so a lost link
// is capped at WARN instead of raising an ERROR nobody needs to act on.
void fillRcLinkStatus(
    const TeleopDiagnostics & diagnostics, bool active,
    diagnostic_updater::DiagnosticStatusWrapper & status);

// "E-Stop requests" task: WARN while any E-Stop service is unreachable or its last request was
// dropped or refused, OK otherwise.
void fillSafetyRequestsStatus(
    const std::vector<SafetyRequestStatus> & requests,
    diagnostic_updater::DiagnosticStatusWrapper & status);

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_INFRASTRUCTURE_TELEOP_DIAGNOSTICS_CONVERSIONS_HPP_
