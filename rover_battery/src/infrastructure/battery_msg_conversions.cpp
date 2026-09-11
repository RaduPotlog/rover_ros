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

#include "rover_battery/infrastructure/battery_msg_conversions.hpp"

#include <string>
#include <vector>

namespace rover_battery::infrastructure
{

namespace
{
constexpr const char * kBatteryLocation = "rover";
}  // namespace

std::uint8_t toPowerSupplyHealth(domain::BatteryHealth health)
{
    switch (health) {
        case domain::BatteryHealth::Good:
            return BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
        case domain::BatteryHealth::Overheat:
            return BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
        case domain::BatteryHealth::Dead:
            return BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
        case domain::BatteryHealth::Overvoltage:
            return BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        case domain::BatteryHealth::Cold:
            return BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD;
        case domain::BatteryHealth::WatchdogTimerExpired:
            return BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
    }
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
}

std::uint8_t toPowerSupplyStatus(domain::ChargeState state)
{
    switch (state) {
        case domain::ChargeState::Unknown:
            return BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
        case domain::ChargeState::Charging:
            return BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
        case domain::ChargeState::Discharging:
            return BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
        case domain::ChargeState::NotCharging:
            return BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING;
        case domain::ChargeState::Full:
            return BatteryStateMsg::POWER_SUPPLY_STATUS_FULL;
    }
    return BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
}

std::uint8_t toChargerType(domain::ChargerType type)
{
    switch (type) {
        case domain::ChargerType::Unknown:
            return ChargingStatusMsg::UNKNOWN;
        case domain::ChargerType::Wired:
            return ChargingStatusMsg::WIRED;
        case domain::ChargerType::Wireless:
            return ChargingStatusMsg::WIRELESS;
    }
    return ChargingStatusMsg::UNKNOWN;
}

BatteryStateMsg toBatteryStateMsg(const domain::BatteryReading & reading)
{
    BatteryStateMsg msg;
    msg.voltage = reading.voltage;
    msg.temperature = reading.temperature;
    msg.current = reading.current;
    msg.charge = reading.charge;
    msg.capacity = reading.capacity;
    msg.design_capacity = reading.design_capacity;
    msg.percentage = reading.percentage;
    msg.power_supply_status = toPowerSupplyStatus(reading.charge_state);
    msg.power_supply_health = toPowerSupplyHealth(reading.health);
    msg.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LIFE;
    msg.present = reading.present;
    msg.cell_voltage = reading.cell_voltages;
    msg.cell_temperature = reading.cell_temperatures;
    msg.location = kBatteryLocation;
    msg.serial_number = reading.serial_number;
    return msg;
}

ChargingStatusMsg toChargingStatusMsg(const domain::ChargingInfo & info)
{
    ChargingStatusMsg msg;
    msg.charging = info.charging;
    msg.current = info.current;
    msg.current_battery = info.battery_current;
    msg.charger_type = toChargerType(info.charger_type);
    return msg;
}

std::string joinErrors(const std::vector<std::string> & errors)
{
    std::string joined;
    for (const auto & error : errors) {
        if (!joined.empty()) {
            joined += '\n';
        }
        joined += error;
    }
    return joined;
}

}  // namespace rover_battery::infrastructure
