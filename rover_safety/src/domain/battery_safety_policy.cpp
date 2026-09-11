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

#include "rover_safety/domain/battery_safety_policy.hpp"

#include <stdexcept>
#include <string>

namespace rover_safety::domain
{

BatteryThresholds::BatteryThresholds(double critical_temp, double fatal_temp)
: critical_temp_(critical_temp)
, fatal_temp_(fatal_temp)
{
    if (!(critical_temp_ < fatal_temp_)) {
        throw std::invalid_argument(
            "Critical battery temperature (" + std::to_string(critical_temp_) +
            ") must be lower than fatal battery temperature (" + std::to_string(fatal_temp_) + ")");
    }
}

BatterySafetyDecision evaluateBatterySafety(
    BatteryHealth health, double temperature, const BatteryThresholds & thresholds)
{
    switch (health) {
        case BatteryHealth::WatchdogTimerExpire:
            return {SafetyVerdict::TripEStop, "Battery watchdog timer expired"};

        case BatteryHealth::Dead:
            return {SafetyVerdict::TripEStop, "Battery dead"};

        case BatteryHealth::Overvoltage:
            return {SafetyVerdict::TripEStop, "Battery overvoltage"};

        case BatteryHealth::Overheat:
            if (temperature > thresholds.fatalTemp()) {
                return {SafetyVerdict::Shutdown, "Fatal battery temperature"};
            }
            if (temperature > thresholds.criticalTemp()) {
                return {SafetyVerdict::TripEStop, "Critical battery temperature"};
            }
            return {};

        default:
            return {};
    }
}

}  // namespace rover_safety::domain
