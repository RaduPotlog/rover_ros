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


#ifndef ROVER_SAFETY_INFRASTRUCTURE_SAFETY_DIAGNOSTICS_HPP_
#define ROVER_SAFETY_INFRASTRUCTURE_SAFETY_DIAGNOSTICS_HPP_

#include <chrono>
#include <optional>
#include <string>

#include <behaviortree_cpp/basic_types.h>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include "rover_safety/domain/safety_health.hpp"

namespace rover_safety::infrastructure
{

using SteadyTime = std::chrono::steady_clock::time_point;

/** Maps a HealthLevel onto a diagnostic_msgs/DiagnosticStatus level byte. */
unsigned char toDiagnosticLevel(domain::HealthLevel level);

/** Seconds since `stamp`, or nullopt when it was never set. */
std::optional<double> ageSeconds(const std::optional<SteadyTime> & stamp, SteadyTime now);

/** Formats evaluateSafetyInputs() plus one "<input> age (s)" value per input. */
void fillSafetyInputsStatus(
    const std::vector<domain::SafetyInput> & inputs,
    diagnostic_updater::DiagnosticStatusWrapper & status);

/**
 * Formats a safety behavior tree's state:
 *  - not configured, or waiting for its inputs (not ticking) -> WARN;
 *  - last tick returned FAILURE -> WARN;
 *  - otherwise -> OK.
 */
void fillBehaviorTreeStatus(
    bool configured, bool ticking, BT::NodeStatus tree_status,
    diagnostic_updater::DiagnosticStatusWrapper & status);

}  // namespace rover_safety::infrastructure

#endif  // ROVER_SAFETY_INFRASTRUCTURE_SAFETY_DIAGNOSTICS_HPP_
