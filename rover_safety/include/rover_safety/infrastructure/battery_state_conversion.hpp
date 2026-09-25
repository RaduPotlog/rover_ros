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

#ifndef ROVER_SAFETY_INFRASTRUCTURE_BATTERY_STATE_CONVERSION_HPP_
#define ROVER_SAFETY_INFRASTRUCTURE_BATTERY_STATE_CONVERSION_HPP_

#include <cstdint>

#include <sensor_msgs/msg/battery_state.hpp>

#include "rover_safety/domain/battery_safety_policy.hpp"
#include "rover_safety/domain/led_animation_policy.hpp"

namespace rover_safety::infrastructure
{

/** sensor_msgs/BatteryState power_supply_health -> domain::BatteryHealth; undefined values are Unknown. */
inline domain::BatteryHealth toBatteryHealth(std::uint8_t power_supply_health)
{
    using BatteryStateMsg = sensor_msgs::msg::BatteryState;
    using domain::BatteryHealth;

    switch (power_supply_health) {
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD: return BatteryHealth::Good;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT: return BatteryHealth::Overheat;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD: return BatteryHealth::Dead;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE: return BatteryHealth::Overvoltage;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: return BatteryHealth::UnspecFailure;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD: return BatteryHealth::Cold;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE:
            return BatteryHealth::WatchdogTimerExpire;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE:
            return BatteryHealth::SafetyTimerExpire;
        default: return BatteryHealth::Unknown;
    }
}

/** sensor_msgs/BatteryState power_supply_status -> domain::PowerSupplyStatus; undefined values are kept. */
inline domain::PowerSupplyStatus toPowerSupplyStatus(std::uint8_t power_supply_status)
{
    return static_cast<domain::PowerSupplyStatus>(power_supply_status);
}

static_assert(
    static_cast<std::uint8_t>(domain::PowerSupplyStatus::Unknown) ==
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
static_assert(
    static_cast<std::uint8_t>(domain::PowerSupplyStatus::Charging) ==
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
static_assert(
    static_cast<std::uint8_t>(domain::PowerSupplyStatus::Discharging) ==
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
static_assert(
    static_cast<std::uint8_t>(domain::PowerSupplyStatus::NotCharging) ==
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING);
static_assert(
    static_cast<std::uint8_t>(domain::PowerSupplyStatus::Full) ==
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL);

}  // namespace rover_safety::infrastructure

#endif  // ROVER_SAFETY_INFRASTRUCTURE_BATTERY_STATE_CONVERSION_HPP_
