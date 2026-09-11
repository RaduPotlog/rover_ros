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

#include <string>
#include <vector>

#include "rover_battery/infrastructure/battery_msg_conversions.hpp"

using namespace rover_battery::infrastructure;  // NOLINT
namespace domain = rover_battery::domain;

TEST(BatteryMsgConversions, HealthMapsToBatteryStateConstants)
{
    EXPECT_EQ(toPowerSupplyHealth(domain::BatteryHealth::Good),
              BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);
    EXPECT_EQ(toPowerSupplyHealth(domain::BatteryHealth::Overheat),
              BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT);
    EXPECT_EQ(toPowerSupplyHealth(domain::BatteryHealth::Dead),
              BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD);
    EXPECT_EQ(toPowerSupplyHealth(domain::BatteryHealth::Overvoltage),
              BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE);
    EXPECT_EQ(toPowerSupplyHealth(domain::BatteryHealth::Cold),
              BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD);
    EXPECT_EQ(toPowerSupplyHealth(domain::BatteryHealth::WatchdogTimerExpired),
              BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE);
}

TEST(BatteryMsgConversions, ChargeStateMapsToBatteryStateConstants)
{
    EXPECT_EQ(toPowerSupplyStatus(domain::ChargeState::Unknown),
              BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN);
    EXPECT_EQ(toPowerSupplyStatus(domain::ChargeState::Charging),
              BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING);
    EXPECT_EQ(toPowerSupplyStatus(domain::ChargeState::Discharging),
              BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING);
    EXPECT_EQ(toPowerSupplyStatus(domain::ChargeState::NotCharging),
              BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING);
    EXPECT_EQ(toPowerSupplyStatus(domain::ChargeState::Full),
              BatteryStateMsg::POWER_SUPPLY_STATUS_FULL);
}

TEST(BatteryMsgConversions, ChargerTypeMapsToChargingStatusConstants)
{
    EXPECT_EQ(toChargerType(domain::ChargerType::Unknown), ChargingStatusMsg::UNKNOWN);
    EXPECT_EQ(toChargerType(domain::ChargerType::Wired), ChargingStatusMsg::WIRED);
    EXPECT_EQ(toChargerType(domain::ChargerType::Wireless), ChargingStatusMsg::WIRELESS);
}

TEST(BatteryMsgConversions, BatteryStateMsgCarriesReading)
{
    domain::BatteryReading reading;
    reading.voltage = 52.0f;
    reading.current = -2.0f;
    reading.percentage = 0.5f;
    reading.design_capacity = 40.0f;
    reading.present = true;
    reading.serial_number = "SN";
    reading.cell_voltages = {3300.0f, 3310.0f};
    reading.cell_temperatures = {21.0f};
    reading.charge_state = domain::ChargeState::Discharging;
    reading.health = domain::BatteryHealth::Cold;

    const auto msg = toBatteryStateMsg(reading);
    EXPECT_FLOAT_EQ(msg.voltage, 52.0f);
    EXPECT_FLOAT_EQ(msg.current, -2.0f);
    EXPECT_FLOAT_EQ(msg.percentage, 0.5f);
    EXPECT_FLOAT_EQ(msg.design_capacity, 40.0f);
    EXPECT_TRUE(msg.present);
    EXPECT_EQ(msg.serial_number, "SN");
    EXPECT_EQ(msg.location, "rover");
    EXPECT_EQ(msg.power_supply_technology, BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LIFE);
    EXPECT_EQ(msg.power_supply_status, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING);
    EXPECT_EQ(msg.power_supply_health, BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD);
    EXPECT_EQ(msg.cell_voltage, reading.cell_voltages);
    EXPECT_EQ(msg.cell_temperature, reading.cell_temperatures);
}

TEST(BatteryMsgConversions, ChargingStatusMsgCarriesInfo)
{
    domain::ChargingInfo info{true, 5.5f, 5.0f, domain::ChargerType::Wired};

    const auto msg = toChargingStatusMsg(info);
    EXPECT_TRUE(msg.charging);
    EXPECT_FLOAT_EQ(msg.current, 5.5f);
    EXPECT_FLOAT_EQ(msg.current_battery, 5.0f);
    EXPECT_EQ(msg.charger_type, ChargingStatusMsg::WIRED);
}

TEST(BatteryMsgConversions, JoinErrors)
{
    EXPECT_EQ(joinErrors({}), "");
    EXPECT_EQ(joinErrors({"a"}), "a");
    EXPECT_EQ(joinErrors({"a", "b", "c"}), "a\nb\nc");
}
