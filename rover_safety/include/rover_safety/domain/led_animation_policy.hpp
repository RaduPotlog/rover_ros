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

#ifndef ROVER_SAFETY_DOMAIN_LED_ANIMATION_POLICY_HPP_
#define ROVER_SAFETY_DOMAIN_LED_ANIMATION_POLICY_HPP_

#include <cstdint>
#include <string>

#include "rover_safety/domain/battery_safety_policy.hpp"

namespace rover_safety::domain
{

/** sensor_msgs/BatteryState power_supply_status. Undefined values stay representable and match none. */
enum class PowerSupplyStatus : std::uint8_t
{
    Unknown = 0,
    Charging = 1,
    Discharging = 2,
    NotCharging = 3,
    Full = 4,
};

/** Which state animation the LEDs request. */
enum class LedStateVerdict : unsigned
{
    Ready = 0,
    ManualAction = 1,
    EStop = 2,
};

/** Which error animation the LEDs request. */
enum class LedErrorVerdict : unsigned
{
    None = 0,
    ChargingOverheat = 1,
    StatusUnknown = 2,
};

/** Which battery animation group applies: charging, discharging, or none at all. */
enum class LedBatteryVerdict : unsigned
{
    None = 0,
    Charging = 1,
    Discharging = 2,
};

struct LedSafetyInputs
{
    PowerSupplyStatus battery_status{PowerSupplyStatus::Unknown};
    BatteryHealth battery_health{BatteryHealth::Unknown};
    /** Last percentage reported with a known status and health; the node latches it. */
    double battery_percent{0.0};
    /** SafetyStatus.hw_e_stop_user_button. */
    bool e_stop_pressed{false};
    /** Joy dead-man button. */
    bool dead_man_held{false};
};

/** Battery fractions in [0, 1]. No validation: critical > low is accepted, as it always was. */
struct LedBatteryThresholds
{
    float critical_percent;
    float low_percent;
    float charging_anim_step;
};

struct LedAnimationDecision
{
    LedStateVerdict state{LedStateVerdict::Ready};
    LedErrorVerdict error{LedErrorVerdict::None};
    LedBatteryVerdict battery{LedBatteryVerdict::None};
    /** The rounded percentage reads 1.0. Consulted only while charging. */
    bool battery_full{false};
    /** critical <= percent < low. The three levels are consulted only while discharging. */
    bool low_battery{false};
    /** percent < critical. */
    bool critical_battery{false};
    /** percent >= low. */
    bool nominal_battery{false};
    /** Percentage rounded to the charging animation step; the battery animations' parameter. */
    std::string battery_percent_round;
};

/** std::to_string(round(percent / step) * step): the text the charging animation shows. */
std::string roundChargingPercent(double battery_percent, float charging_anim_step);

/**
 * Decides the LED safety animations from the rover's inputs. Each channel is independent:
 *  - state: E-Stop pressed -> EStop, else dead-man held -> ManualAction, else Ready;
 *  - error: status unknown -> StatusUnknown, charging while overheating -> ChargingOverheat,
 *    else None (FULL counts as charging for the battery channel only);
 *  - battery: charging or full -> Charging, discharging or not charging -> Discharging, else None;
 *  - levels compare the percentage as a float, so a NaN percentage selects none of them.
 * It reproduces the BehaviorTree.CPP script expressions rover_led_safety.xml used to evaluate.
 */
LedAnimationDecision evaluateLedAnimation(
    const LedSafetyInputs & inputs, const LedBatteryThresholds & thresholds);

}  // namespace rover_safety::domain

#endif  // ROVER_SAFETY_DOMAIN_LED_ANIMATION_POLICY_HPP_
