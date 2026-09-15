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

#ifndef ROVER_BATTERY_DOMAIN_BATTERY_REPORT_HPP_
#define ROVER_BATTERY_DOMAIN_BATTERY_REPORT_HPP_

#include <string>
#include <vector>

namespace rover_battery::domain
{

enum class BatteryHealth
{
    Good,
    Overheat,
    Dead,
    Overvoltage,
    Cold,
    WatchdogTimerExpired,
};

enum class ChargeState
{
    Unknown,
    Charging,
    Discharging,
    NotCharging,
    Full,
};

enum class ChargerType
{
    Unknown,
    Wired,
    Wireless,
};

/** @brief Static facts about the installed pack (configured per robot, not reported by the BMS). */
struct BatteryIdentity
{
    float design_capacity{40.0f};  // [Ah]
    std::string serial_number{"224KA141600043"};
};

/**
 * @brief Battery snapshot in sensor_msgs/BatteryState units; NaN = not measured.
 */
struct BatteryReading
{
    float voltage{0.0f};          // [V]
    float temperature{0.0f};      // [°C]
    float current{0.0f};          // [A], negative while discharging
    float charge{0.0f};           // [Ah] remaining
    float capacity{0.0f};         // [Ah] last full capacity
    float design_capacity{0.0f};  // [Ah]
    float percentage{0.0f};       // [0, 1]
    bool present{false};
    std::string serial_number;
    std::vector<float> cell_voltages;      // [V]
    std::vector<float> cell_temperatures;  // [°C]
    ChargeState charge_state{ChargeState::Unknown};
    BatteryHealth health{BatteryHealth::Good};
};

struct ChargingInfo
{
    bool charging{false};
    float current{0.0f};
    float battery_current{0.0f};
    ChargerType charger_type{ChargerType::Unknown};
};

/** @brief Everything published for one BMS update (or one watchdog expiry). */
struct BatteryReport
{
    BatteryReading reading;
    ChargingInfo charging;
    std::vector<std::string> errors;  // empty = no active alarms
};

}  // namespace rover_battery::domain

#endif  // ROVER_BATTERY_DOMAIN_BATTERY_REPORT_HPP_
