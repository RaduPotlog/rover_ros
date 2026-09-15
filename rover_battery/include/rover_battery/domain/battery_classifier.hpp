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

#ifndef ROVER_BATTERY_DOMAIN_BATTERY_CLASSIFIER_HPP_
#define ROVER_BATTERY_DOMAIN_BATTERY_CLASSIFIER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "rover_battery/domain/battery_report.hpp"
#include "rover_battery/domain/bms_frame.hpp"

namespace rover_battery::domain
{

/** Percentage at or above which a charging pack is reported as full. */
constexpr float kFullChargeThreshold = 1.0f;

constexpr const char * kWatchdogExpiredError = "Battery watchdog expired";

/**
 * True for the all-zero payload the BMS bridge sends when it has no BMS data (BLE link down
 * or the BMS stopped answering). Such a frame must not be decoded as a reading.
 */
bool isNoDataFrame(const BmsFrame & frame);

/** Overvoltage/dead from voltage & SoC alarms; a temperature alarm overrides either. */
BatteryHealth classifyBatteryHealth(const BmsAlarms & alarms);

/** Names of every active alarm, in BMS bit order. */
std::vector<std::string> describeAlarms(const BmsAlarms & alarms);

/** Maps the BMS charge/discharge status (0 idle, 1 charge, 2 discharge) to a charge state. */
ChargeState classifyChargeState(const BmsData & data);

ChargingInfo toChargingInfo(const BmsData & data);

/** Cell / sensor counts reported by the BMS, clamped to the payload's array sizes. */
std::size_t validCellCount(const BmsData & data);
std::size_t validTempSensorCount(const BmsData & data);

/**
 * Converts a BMS frame to a battery reading (health and charge state included).
 * temperature is the hottest sensor, so rover_safety's thresholds see a single hot cell.
 */
BatteryReading toBatteryReading(const BmsFrame & frame, const BatteryIdentity & identity);

/** Full report for one BMS frame. */
BatteryReport buildBatteryReport(const BmsFrame & frame, const BatteryIdentity & identity);

/** Report published when no BMS data arrived within the watchdog timeout. */
BatteryReport staleBatteryReport(
    const BatteryIdentity & identity, std::size_t cell_count, std::size_t temp_sensor_count);

}  // namespace rover_battery::domain

#endif  // ROVER_BATTERY_DOMAIN_BATTERY_CLASSIFIER_HPP_
