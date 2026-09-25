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

#include "rover_safety/domain/led_animation_policy.hpp"

#include <cfloat>
#include <charconv>
#include <cmath>
#include <string>
#include <system_error>

namespace rover_safety::domain
{

namespace
{

LedStateVerdict stateVerdict(const LedSafetyInputs & inputs)
{
    if (inputs.e_stop_pressed) {
        return LedStateVerdict::EStop;
    }
    return inputs.dead_man_held ? LedStateVerdict::ManualAction : LedStateVerdict::Ready;
}

LedErrorVerdict errorVerdict(const LedSafetyInputs & inputs)
{
    if (inputs.battery_status == PowerSupplyStatus::Unknown) {
        return LedErrorVerdict::StatusUnknown;
    }
    if (inputs.battery_status == PowerSupplyStatus::Charging &&
        inputs.battery_health == BatteryHealth::Overheat) {
        return LedErrorVerdict::ChargingOverheat;
    }
    return LedErrorVerdict::None;
}

LedBatteryVerdict batteryVerdict(PowerSupplyStatus status)
{
    switch (status) {
        case PowerSupplyStatus::Charging:
        case PowerSupplyStatus::Full:
            return LedBatteryVerdict::Charging;

        case PowerSupplyStatus::Discharging:
        case PowerSupplyStatus::NotCharging:
            return LedBatteryVerdict::Discharging;

        default:
            return LedBatteryVerdict::None;
    }
}

/**
 * The tree compared the rounded text with 1.0: BT.CPP parses the string with std::from_chars and
 * treats numbers within float epsilon as equal.
 */
bool readsOne(const std::string & text)
{
    double value = 0.0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    return result.ec == std::errc() && std::abs(value - 1.0) <= static_cast<double>(FLT_EPSILON);
}

}  // namespace

std::string roundChargingPercent(double battery_percent, float charging_anim_step)
{
    return std::to_string(std::round(battery_percent / charging_anim_step) * charging_anim_step);
}

LedAnimationDecision evaluateLedAnimation(
    const LedSafetyInputs & inputs, const LedBatteryThresholds & thresholds)
{
    LedAnimationDecision decision;
    decision.state = stateVerdict(inputs);
    decision.error = errorVerdict(inputs);
    decision.battery = batteryVerdict(inputs.battery_status);

    decision.battery_percent_round =
        roundChargingPercent(inputs.battery_percent, thresholds.charging_anim_step);
    decision.battery_full = readsOne(decision.battery_percent_round);

    // The blackboard held the percentage as a float. Written as plain comparisons, a NaN makes
    // every level false, which is what the tree's negated `<` / `>=` checks did.
    const float percent = static_cast<float>(inputs.battery_percent);
    decision.critical_battery = percent < thresholds.critical_percent;
    decision.low_battery = percent >= thresholds.critical_percent && percent < thresholds.low_percent;
    decision.nominal_battery = percent >= thresholds.low_percent;

    return decision;
}

}  // namespace rover_safety::domain
