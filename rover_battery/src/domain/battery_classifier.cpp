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

#include "rover_battery/domain/battery_classifier.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace rover_battery::domain
{

namespace
{

// Bitfields cannot be addressed with pointers-to-member, hence one accessor per alarm.
using AlarmAccessor = bool (*)(const BmsAlarms &);

struct AlarmEntry
{
    const char * name;
    AlarmAccessor is_active;
};

#define ROVER_BATTERY_ALARM(field) \
    AlarmEntry{#field, [](const BmsAlarms & a) -> bool {return a.field;}}

constexpr std::array kAlarmTable{
    ROVER_BATTERY_ALARM(levelOneCellVoltageTooHigh),
    ROVER_BATTERY_ALARM(levelTwoCellVoltageTooHigh),
    ROVER_BATTERY_ALARM(levelOneCellVoltageTooLow),
    ROVER_BATTERY_ALARM(levelTwoCellVoltageTooLow),
    ROVER_BATTERY_ALARM(levelOnePackVoltageTooHigh),
    ROVER_BATTERY_ALARM(levelTwoPackVoltageTooHigh),
    ROVER_BATTERY_ALARM(levelOnePackVoltageTooLow),
    ROVER_BATTERY_ALARM(levelTwoPackVoltageTooLow),

    ROVER_BATTERY_ALARM(levelOneChargeTempTooHigh),
    ROVER_BATTERY_ALARM(levelTwoChargeTempTooHigh),
    ROVER_BATTERY_ALARM(levelOneChargeTempTooLow),
    ROVER_BATTERY_ALARM(levelTwoChargeTempTooLow),
    ROVER_BATTERY_ALARM(levelOneDischargeTempTooHigh),
    ROVER_BATTERY_ALARM(levelTwoDischargeTempTooHigh),
    ROVER_BATTERY_ALARM(levelOneDischargeTempTooLow),
    ROVER_BATTERY_ALARM(levelTwoDischargeTempTooLow),

    ROVER_BATTERY_ALARM(levelOneChargeCurrentTooHigh),
    ROVER_BATTERY_ALARM(levelTwoChargeCurrentTooHigh),
    ROVER_BATTERY_ALARM(levelOneDischargeCurrentTooHigh),
    ROVER_BATTERY_ALARM(levelTwoDischargeCurrentTooHigh),
    ROVER_BATTERY_ALARM(levelOneStateOfChargeTooHigh),
    ROVER_BATTERY_ALARM(levelTwoStateOfChargeTooHigh),
    ROVER_BATTERY_ALARM(levelOneStateOfChargeTooLow),
    ROVER_BATTERY_ALARM(levelTwoStateOfChargeTooLow),

    ROVER_BATTERY_ALARM(levelOneCellVoltageDifferenceTooHigh),
    ROVER_BATTERY_ALARM(levelTwoCellVoltageDifferenceTooHigh),
    ROVER_BATTERY_ALARM(levelOneTempSensorDifferenceTooHigh),
    ROVER_BATTERY_ALARM(levelTwoTempSensorDifferenceTooHigh),

    ROVER_BATTERY_ALARM(chargeFETTemperatureTooHigh),
    ROVER_BATTERY_ALARM(dischargeFETTemperatureTooHigh),
    ROVER_BATTERY_ALARM(failureOfChargeFETTemperatureSensor),
    ROVER_BATTERY_ALARM(failureOfDischargeFETTemperatureSensor),
    ROVER_BATTERY_ALARM(failureOfChargeFETAdhesion),
    ROVER_BATTERY_ALARM(failureOfDischargeFETAdhesion),
    ROVER_BATTERY_ALARM(failureOfChargeFETTBreaker),
    ROVER_BATTERY_ALARM(failureOfDischargeFETBreaker),

    ROVER_BATTERY_ALARM(failureOfAFEAcquisitionModule),
    ROVER_BATTERY_ALARM(failureOfVoltageSensorModule),
    ROVER_BATTERY_ALARM(failureOfTemperatureSensorModule),
    ROVER_BATTERY_ALARM(failureOfEEPROMStorageModule),
    ROVER_BATTERY_ALARM(failureOfRealtimeClockModule),
    ROVER_BATTERY_ALARM(failureOfPrechargeModule),
    ROVER_BATTERY_ALARM(failureOfVehicleCommunicationModule),
    ROVER_BATTERY_ALARM(failureOfIntranetCommunicationModule),

    ROVER_BATTERY_ALARM(failureOfCurrentSensorModule),
    ROVER_BATTERY_ALARM(failureOfMainVoltageSensorModule),
    ROVER_BATTERY_ALARM(failureOfShortCircuitProtection),
    ROVER_BATTERY_ALARM(failureOfLowVoltageNoCharging),
};

#undef ROVER_BATTERY_ALARM

std::size_t clampCount(int reported, std::size_t max)
{
    return reported <= 0 ? 0 : std::min(static_cast<std::size_t>(reported), max);
}

}  // namespace

BatteryHealth classifyBatteryHealth(const BmsAlarms & alarms)
{
    BatteryHealth health = BatteryHealth::Good;

    if (alarms.levelOnePackVoltageTooLow || alarms.levelOneCellVoltageTooLow ||
        alarms.levelOneStateOfChargeTooLow)
    {
        health = BatteryHealth::Dead;
    } else if (alarms.levelTwoPackVoltageTooHigh || alarms.levelTwoStateOfChargeTooHigh) {
        health = BatteryHealth::Overvoltage;
    }

    if (alarms.levelOneChargeTempTooHigh || alarms.levelOneDischargeTempTooHigh) {
        health = BatteryHealth::Overheat;
    } else if (alarms.levelOneChargeTempTooLow || alarms.levelOneDischargeTempTooLow) {
        health = BatteryHealth::Cold;
    }

    return health;
}

std::vector<std::string> describeAlarms(const BmsAlarms & alarms)
{
    std::vector<std::string> active;

    for (const auto & entry : kAlarmTable) {
        if (entry.is_active(alarms)) {
            active.emplace_back(entry.name);
        }
    }

    return active;
}

ChargeState classifyChargeState(const BmsData & data)
{
    switch (data.chargeDischargeStatus) {
        case 0:
            return ChargeState::NotCharging;
        case 1:
            return (data.packSOC / 100.0f) >= kFullChargeThreshold ?
                   ChargeState::Full : ChargeState::Charging;
        case 2:
            return ChargeState::Discharging;
        default:
            return ChargeState::Unknown;
    }
}

ChargingInfo toChargingInfo(const BmsData & data)
{
    ChargingInfo info;
    info.charging = data.chargeDischargeStatus == 1;
    info.current = data.packCurrent;
    info.battery_current = data.packCurrent;
    info.charger_type = (data.chargeDischargeStatus == 1 || data.chargeDischargeStatus == 2) ?
                        ChargerType::Wired : ChargerType::Unknown;
    return info;
}

std::size_t validCellCount(const BmsData & data)
{
    return clampCount(data.numberOfCells, kBmsMaxCells);
}

std::size_t validTempSensorCount(const BmsData & data)
{
    return clampCount(data.numOfTempSensors, kBmsMaxTempSensors);
}

BatteryReading toBatteryReading(const BmsFrame & frame, const BatteryIdentity & identity)
{
    const BmsData & data = frame.data;

    BatteryReading reading;
    reading.voltage = data.packVoltage;
    reading.temperature = data.tempAverage;
    reading.current = data.packCurrent;
    reading.charge = data.packSOC;
    reading.capacity = static_cast<float>(data.resCapacitymAh);
    reading.design_capacity = identity.design_capacity;
    reading.percentage = data.packSOC / 100.0f;
    reading.present = true;
    reading.serial_number = identity.serial_number;

    // Element-wise copies: the arrays are members of a packed struct and may be unaligned.
    const std::size_t cell_count = validCellCount(data);
    reading.cell_voltages.reserve(cell_count);
    for (std::size_t i = 0; i < cell_count; ++i) {
        reading.cell_voltages.push_back(data.cellVmV[i]);
    }

    const std::size_t temp_count = validTempSensorCount(data);
    reading.cell_temperatures.reserve(temp_count);
    for (std::size_t i = 0; i < temp_count; ++i) {
        reading.cell_temperatures.push_back(static_cast<float>(data.cellTemperature[i]));
    }

    reading.charge_state = classifyChargeState(data);
    reading.health = classifyBatteryHealth(frame.alarms);
    return reading;
}

BatteryReport buildBatteryReport(const BmsFrame & frame, const BatteryIdentity & identity)
{
    BatteryReport report;
    report.reading = toBatteryReading(frame, identity);
    report.charging = toChargingInfo(frame.data);
    report.errors = describeAlarms(frame.alarms);
    return report;
}

BatteryReport staleBatteryReport(
    const BatteryIdentity & identity, std::size_t cell_count, std::size_t temp_sensor_count)
{
    BatteryReport report;
    report.reading.design_capacity = identity.design_capacity;
    report.reading.present = false;
    report.reading.serial_number = identity.serial_number;
    report.reading.cell_voltages.assign(std::min(cell_count, kBmsMaxCells), 0.0f);
    report.reading.cell_temperatures.assign(std::min(temp_sensor_count, kBmsMaxTempSensors), 0.0f);
    // TODO(rover_battery): Full is inherited behaviour; Unknown would be more accurate for stale
    // data but rover_safety consumers must be checked first (see README "Known issues").
    report.reading.charge_state = ChargeState::Full;
    report.reading.health = BatteryHealth::WatchdogTimerExpired;
    report.errors = {kWatchdogExpiredError};
    return report;
}

}  // namespace rover_battery::domain
