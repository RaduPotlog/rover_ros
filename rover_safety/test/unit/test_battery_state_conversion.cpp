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

#include <gtest/gtest.h>

#include <cstdint>

#include <sensor_msgs/msg/battery_state.hpp>

#include "rover_safety/infrastructure/battery_state_conversion.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using rover_safety::domain::BatteryHealth;
using rover_safety::domain::PowerSupplyStatus;
using rover_safety::infrastructure::toBatteryHealth;
using rover_safety::infrastructure::toPowerSupplyStatus;

TEST(BatteryStateConversion, MapsEveryHealthValue)
{
    EXPECT_EQ(toBatteryHealth(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN), BatteryHealth::Unknown);
    EXPECT_EQ(toBatteryHealth(BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD), BatteryHealth::Good);
    EXPECT_EQ(toBatteryHealth(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT), BatteryHealth::Overheat);
    EXPECT_EQ(toBatteryHealth(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD), BatteryHealth::Dead);
    EXPECT_EQ(toBatteryHealth(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE), BatteryHealth::Overvoltage);
    EXPECT_EQ(
        toBatteryHealth(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE), BatteryHealth::UnspecFailure);
    EXPECT_EQ(toBatteryHealth(BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD), BatteryHealth::Cold);
    EXPECT_EQ(
        toBatteryHealth(BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE),
        BatteryHealth::WatchdogTimerExpire);
    EXPECT_EQ(
        toBatteryHealth(BatteryStateMsg::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE),
        BatteryHealth::SafetyTimerExpire);

    EXPECT_EQ(toBatteryHealth(9), BatteryHealth::Unknown);
    EXPECT_EQ(toBatteryHealth(255), BatteryHealth::Unknown);
}

TEST(BatteryStateConversion, KeepsStatusValue)
{
    EXPECT_EQ(toPowerSupplyStatus(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN), PowerSupplyStatus::Unknown);
    EXPECT_EQ(toPowerSupplyStatus(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING), PowerSupplyStatus::Charging);
    EXPECT_EQ(
        toPowerSupplyStatus(BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING), PowerSupplyStatus::Discharging);
    EXPECT_EQ(
        toPowerSupplyStatus(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING), PowerSupplyStatus::NotCharging);
    EXPECT_EQ(toPowerSupplyStatus(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL), PowerSupplyStatus::Full);

    // An undefined status is kept, and matches no enumerator.
    EXPECT_EQ(static_cast<std::uint8_t>(toPowerSupplyStatus(7)), 7);
}
