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


#include "rover_safety/domain/safety_health.hpp"

#include <string>
#include <vector>

namespace rover_safety::domain
{

namespace
{

std::string join(const std::vector<std::string> & names)
{
    std::string joined;
    for (const auto & name : names) {
        joined += (joined.empty() ? "" : ", ") + name;
    }
    return joined;
}

}  // namespace

SafetyInputsHealth evaluateSafetyInputs(const std::vector<SafetyInput> & inputs)
{
    SafetyInputsHealth health;

    for (const auto & input : inputs) {
        if (!input.age_s.has_value()) {
            health.missing.push_back(input.name);
        } else if (input.timeout_s.has_value() && *input.age_s > *input.timeout_s) {
            health.stale.push_back(input.name);
        }
    }

    if (!health.stale.empty()) {
        health.level = HealthLevel::Error;
        health.message = "Stale safety inputs: " + join(health.stale) + ".";
    } else if (!health.missing.empty()) {
        health.level = HealthLevel::Warn;
        health.message = "Waiting for safety inputs: " + join(health.missing) + ".";
    } else {
        health.message = "All safety inputs are fresh.";
    }

    return health;
}

HealthLevel verdictHealthLevel(const SafetyVerdict verdict)
{
    return verdict == SafetyVerdict::None ? HealthLevel::Ok : HealthLevel::Error;
}

const char * toString(const SafetyVerdict verdict)
{
    switch (verdict) {
        case SafetyVerdict::None: return "none";
        case SafetyVerdict::TripEStop: return "trip E-Stop";
        case SafetyVerdict::Shutdown: return "shutdown";
    }
    return "unknown";
}

}  // namespace rover_safety::domain
