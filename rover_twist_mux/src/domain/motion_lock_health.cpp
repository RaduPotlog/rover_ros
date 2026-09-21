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


#include "rover_twist_mux/domain/motion_lock_health.hpp"

#include <cstdio>
#include <string>

namespace rover_twist_mux::domain
{

MotionLockHealth evaluateMotionLockHealth(
    const std::optional<SafetyIoFlags> & flags,
    const double gpio_age_s,
    const double gpio_timeout_s,
    const MotionLockPolicy & policy,
    const bool link_healthy)
{
    MotionLockHealth health;

    // Nothing received yet: deny motion rather than assume the rover is safe to drive.
    if (!flags.has_value()) {
        health.message = "No safety state received yet: motion locked.";
        return health;
    }

    if (gpio_age_s > gpio_timeout_s) {
        char buffer[128];
        std::snprintf(
            buffer, sizeof(buffer), "safety state stale (%.2f s > %.2f s): motion locked.",
            gpio_age_s, gpio_timeout_s);
        health.message = buffer;
        return health;
    }

    health.reasons = motionInhibitReasons(*flags, policy);

    // A down link is an input-trust failure, not a stop condition, so it is graded Error like
    // staleness rather than Warn - but unlike staleness the messages keep flowing, so nothing
    // else would have caught it.
    if (!link_healthy) {
        health.reasons.push_back(MotionInhibitReason::SafetyLinkUnhealthy);
        health.message = "Safety PLC link unhealthy: motion locked.";
        return health;
    }

    if (health.reasons.empty()) {
        health.level = HealthLevel::Ok;
        health.locked = false;
        health.message = "Motion permitted.";
        return health;
    }

    health.level = HealthLevel::Warn;
    health.message = "Motion locked: ";
    for (std::size_t i = 0; i < health.reasons.size(); ++i) {
        health.message += (i == 0 ? "" : ", ");
        health.message += toString(health.reasons[i]);
    }
    health.message += ".";

    return health;
}

}  // namespace rover_twist_mux::domain
