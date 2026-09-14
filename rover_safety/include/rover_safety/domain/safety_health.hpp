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


#ifndef ROVER_SAFETY_DOMAIN_SAFETY_HEALTH_HPP_
#define ROVER_SAFETY_DOMAIN_SAFETY_HEALTH_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rover_safety/domain/battery_safety_policy.hpp"

namespace rover_safety::domain
{

/** Ordered by severity. */
enum class HealthLevel
{
    Ok,
    Warn,
    Error,
};

/** One input the safety tree depends on. */
struct SafetyInput
{
    std::string name;
    /** Seconds since the last message; nullopt when nothing arrived yet. */
    std::optional<double> age_s;
    /**
     * How long a message stays trusted; nullopt for inputs published only on change (e.g. the
     * transient-local gpio_state), which can never go stale.
     */
    std::optional<double> timeout_s;
};

struct SafetyInputsHealth
{
    HealthLevel level{HealthLevel::Ok};
    std::string message;
    std::vector<std::string> missing;
    std::vector<std::string> stale;
};

/**
 * Grades the inputs of a safety tree:
 *  - any input stale -> Error (the tree is deciding on outdated data);
 *  - any input never received -> Warn (the tree is not ticking yet);
 *  - otherwise -> Ok.
 */
SafetyInputsHealth evaluateSafetyInputs(const std::vector<SafetyInput> & inputs);

/** None -> Ok; TripEStop and Shutdown -> Error: the robot is being stopped by the safety layer. */
HealthLevel verdictHealthLevel(SafetyVerdict verdict);

const char * toString(SafetyVerdict verdict);

}  // namespace rover_safety::domain

#endif  // ROVER_SAFETY_DOMAIN_SAFETY_HEALTH_HPP_
