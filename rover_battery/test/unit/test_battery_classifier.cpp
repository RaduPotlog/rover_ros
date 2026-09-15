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

#include <cmath>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "rover_battery/domain/battery_classifier.hpp"

using namespace rover_battery::domain;  // NOLINT

namespace
{

BmsFrame makeFrame(float soc, int status, int cells = 16, int temps = 4)
{
    BmsFrame frame;
    frame.data.packVoltage = 52.1f;
    frame.data.packCurrent = -3.5f;
    frame.data.packSOC = soc;
    frame.data.tempMax = 31.0f;
    frame.data.tempMin = 20.0f;
    frame.data.tempAverage = 25.5f;
    frame.data.resCapacitymAh = 32000;
    frame.data.chargeDischargeStatus = status;
    frame.data.numberOfCells = cells;
    frame.data.numOfTempSensors = temps;
    for (std::size_t i = 0; i < kBmsMaxCells; ++i) {
        frame.data.cellVmV[i] = 3000.0f + static_cast<float>(i);
    }
    for (std::size_t i = 0; i < kBmsMaxTempSensors; ++i) {
        frame.data.cellTemperature[i] = 20 + static_cast<int>(i);
    }
    return frame;
}

/** Sets bit `bit` of byte `byte` in the wire representation of the alarms. */
BmsAlarms alarmsWithBits(std::initializer_list<std::pair<int, int>> bits)
{
    std::uint8_t raw[sizeof(BmsAlarms)] = {};
    for (const auto & [byte, bit] : bits) {
        raw[byte] |= static_cast<std::uint8_t>(1U << bit);
    }
    BmsAlarms alarms{};
    std::memcpy(&alarms, raw, sizeof(raw));
    return alarms;
}

}  // namespace

TEST(BmsFrame, WireLayoutIsStable)
{
    EXPECT_EQ(kBmsPayloadSize, 392u);
}

TEST(IsNoDataFrame, AllZeroPayloadHasNoData)
{
    EXPECT_TRUE(isNoDataFrame(BmsFrame{}));
}

TEST(IsNoDataFrame, AnyDataOrAlarmIsARealFrame)
{
    BmsFrame with_voltage{};
    with_voltage.data.packVoltage = 51.2f;
    EXPECT_FALSE(isNoDataFrame(with_voltage));

    BmsFrame with_alarm{};
    with_alarm.alarms = alarmsWithBits({{6, 3}});
    EXPECT_FALSE(isNoDataFrame(with_alarm));

    EXPECT_FALSE(isNoDataFrame(makeFrame(50.0f, 0)));
}

TEST(ClassifyBatteryHealth, NoAlarmsIsGood)
{
    EXPECT_EQ(classifyBatteryHealth(BmsAlarms{}), BatteryHealth::Good);
}

TEST(ClassifyBatteryHealth, LowVoltageOrSocIsDead)
{
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{0, 6}})), BatteryHealth::Dead);  // pack V
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{0, 2}})), BatteryHealth::Dead);  // cell V
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{2, 6}})), BatteryHealth::Dead);  // SoC
}

TEST(ClassifyBatteryHealth, LevelTwoHighIsOvervoltage)
{
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{0, 5}})), BatteryHealth::Overvoltage);
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{2, 5}})), BatteryHealth::Overvoltage);
    // level-one high alarms alone do not change health
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{0, 4}})), BatteryHealth::Good);
}

TEST(ClassifyBatteryHealth, DeadTakesPriorityOverOvervoltage)
{
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{0, 6}, {0, 5}})), BatteryHealth::Dead);
}

TEST(ClassifyBatteryHealth, TemperatureOverridesVoltage)
{
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{0, 6}, {1, 0}})), BatteryHealth::Overheat);
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{0, 5}, {1, 4}})), BatteryHealth::Overheat);
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{1, 2}})), BatteryHealth::Cold);
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{1, 6}})), BatteryHealth::Cold);
    // overheat wins over cold
    EXPECT_EQ(classifyBatteryHealth(alarmsWithBits({{1, 0}, {1, 2}})), BatteryHealth::Overheat);
}

TEST(DescribeAlarms, EmptyWhenNoAlarms)
{
    EXPECT_TRUE(describeAlarms(BmsAlarms{}).empty());
}

TEST(DescribeAlarms, ListsActiveAlarmsInBitOrder)
{
    const auto errors = describeAlarms(alarmsWithBits({{6, 3}, {0, 2}, {2, 6}}));
    const std::vector<std::string> expected{
        "levelOneCellVoltageTooLow", "levelOneStateOfChargeTooLow",
        "failureOfLowVoltageNoCharging"};
    EXPECT_EQ(errors, expected);
}

TEST(DescribeAlarms, ReportsLevelTwoCellVoltageTooHigh)
{
    // Regression: the node used to test levelOneCellVoltageTooHigh for this message.
    EXPECT_EQ(describeAlarms(alarmsWithBits({{0, 1}})),
              std::vector<std::string>{"levelTwoCellVoltageTooHigh"});
    EXPECT_EQ(describeAlarms(alarmsWithBits({{0, 0}})),
              std::vector<std::string>{"levelOneCellVoltageTooHigh"});
}

TEST(DescribeAlarms, EveryDefinedBitHasAName)
{
    std::uint8_t raw[sizeof(BmsAlarms)];
    std::memset(raw, 0xFF, sizeof(raw));
    BmsAlarms all{};
    std::memcpy(&all, raw, sizeof(raw));
    // 7 bytes * 8 bits minus the two 4-bit padding groups
    EXPECT_EQ(describeAlarms(all).size(), 48u);
}

TEST(ClassifyChargeState, MapsBmsStatus)
{
    EXPECT_EQ(classifyChargeState(makeFrame(50.0f, 0).data), ChargeState::NotCharging);
    EXPECT_EQ(classifyChargeState(makeFrame(50.0f, 1).data), ChargeState::Charging);
    EXPECT_EQ(classifyChargeState(makeFrame(50.0f, 2).data), ChargeState::Discharging);
    EXPECT_EQ(classifyChargeState(makeFrame(50.0f, 3).data), ChargeState::Unknown);
    EXPECT_EQ(classifyChargeState(makeFrame(50.0f, -1).data), ChargeState::Unknown);
}

TEST(ClassifyChargeState, ChargingAtFullThresholdIsFull)
{
    EXPECT_EQ(classifyChargeState(makeFrame(99.9f, 1).data), ChargeState::Charging);
    EXPECT_EQ(classifyChargeState(makeFrame(100.0f, 1).data), ChargeState::Full);
    // Full is only reported while charging
    EXPECT_EQ(classifyChargeState(makeFrame(100.0f, 0).data), ChargeState::NotCharging);
}

TEST(ToChargingInfo, MapsBmsStatus)
{
    const auto charging = toChargingInfo(makeFrame(50.0f, 1).data);
    EXPECT_TRUE(charging.charging);
    EXPECT_EQ(charging.charger_type, ChargerType::Wired);
    EXPECT_FLOAT_EQ(charging.current, -3.5f);
    EXPECT_FLOAT_EQ(charging.battery_current, -3.5f);

    // Discharging means no charger is attached, so the type must not claim one.
    const auto discharging = toChargingInfo(makeFrame(50.0f, 2).data);
    EXPECT_FALSE(discharging.charging);
    EXPECT_EQ(discharging.charger_type, ChargerType::Unknown);

    EXPECT_EQ(toChargingInfo(makeFrame(50.0f, 0).data).charger_type, ChargerType::Unknown);
    EXPECT_EQ(toChargingInfo(makeFrame(50.0f, 7).data).charger_type, ChargerType::Unknown);
}

TEST(ToBatteryReading, ConvertsBmsValues)
{
    const BatteryIdentity identity{42.0f, "SN-1"};
    const auto reading = toBatteryReading(makeFrame(80.0f, 2), identity);

    EXPECT_FLOAT_EQ(reading.voltage, 52.1f);
    EXPECT_FLOAT_EQ(reading.current, -3.5f);
    EXPECT_FLOAT_EQ(reading.temperature, 31.0f);  // hottest sensor, not the average
    EXPECT_FLOAT_EQ(reading.charge, 32.0f);  // 32000 mAh
    EXPECT_FLOAT_EQ(reading.percentage, 0.8f);
    EXPECT_TRUE(std::isnan(reading.capacity));
    EXPECT_FLOAT_EQ(reading.design_capacity, 42.0f);
    EXPECT_EQ(reading.serial_number, "SN-1");
    EXPECT_TRUE(reading.present);
    EXPECT_EQ(reading.charge_state, ChargeState::Discharging);
    EXPECT_EQ(reading.health, BatteryHealth::Good);

    ASSERT_EQ(reading.cell_voltages.size(), 16u);
    EXPECT_FLOAT_EQ(reading.cell_voltages.front(), 3.0f);  // mV -> V
    EXPECT_FLOAT_EQ(reading.cell_voltages.back(), 3.015f);
    ASSERT_EQ(reading.cell_temperatures.size(), 4u);
    EXPECT_FLOAT_EQ(reading.cell_temperatures.back(), 23.0f);
}

TEST(ToBatteryReading, ClampsCountsReportedByBms)
{
    const auto too_many = toBatteryReading(makeFrame(50.0f, 0, 1000, 1000), BatteryIdentity{});
    EXPECT_EQ(too_many.cell_voltages.size(), kBmsMaxCells);
    EXPECT_EQ(too_many.cell_temperatures.size(), kBmsMaxTempSensors);

    const auto negative = toBatteryReading(makeFrame(50.0f, 0, -5, -1), BatteryIdentity{});
    EXPECT_TRUE(negative.cell_voltages.empty());
    EXPECT_TRUE(negative.cell_temperatures.empty());
}

TEST(BuildBatteryReport, CombinesReadingChargingAndErrors)
{
    auto frame = makeFrame(5.0f, 2);
    frame.alarms = alarmsWithBits({{2, 6}});

    const auto report = buildBatteryReport(frame, BatteryIdentity{});
    EXPECT_EQ(report.reading.health, BatteryHealth::Dead);
    EXPECT_EQ(report.charging.charger_type, ChargerType::Unknown);
    EXPECT_EQ(report.errors, std::vector<std::string>{"levelOneStateOfChargeTooLow"});
}

TEST(StaleBatteryReport, ReportsWatchdogExpiry)
{
    const BatteryIdentity identity{40.0f, "SN-2"};
    const auto report = staleBatteryReport(identity, 16, 4);

    EXPECT_FALSE(report.reading.present);
    EXPECT_EQ(report.reading.health, BatteryHealth::WatchdogTimerExpired);
    EXPECT_EQ(report.reading.charge_state, ChargeState::Unknown);
    EXPECT_TRUE(std::isnan(report.reading.charge));
    EXPECT_TRUE(std::isnan(report.reading.capacity));
    EXPECT_FLOAT_EQ(report.reading.voltage, 0.0f);
    EXPECT_FLOAT_EQ(report.reading.percentage, 0.0f);
    EXPECT_FLOAT_EQ(report.reading.design_capacity, 40.0f);
    EXPECT_EQ(report.reading.serial_number, "SN-2");
    EXPECT_EQ(report.reading.cell_voltages, std::vector<float>(16, 0.0f));
    EXPECT_EQ(report.reading.cell_temperatures, std::vector<float>(4, 0.0f));
    EXPECT_FALSE(report.charging.charging);
    EXPECT_EQ(report.charging.charger_type, ChargerType::Unknown);
    EXPECT_EQ(report.errors, std::vector<std::string>{kWatchdogExpiredError});
}
