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

#include <memory>
#include <stdexcept>
#include <vector>

#include "rover_battery/application/monitor_battery_use_case.hpp"

using rover_battery::application::MonitorBatteryUseCase;
using namespace rover_battery::domain;  // NOLINT

namespace
{

class FakePublisher : public BatteryStatePublisherPort
{
public:
    void publish(const BatteryReport & report) override {reports.push_back(report);}

    std::vector<BatteryReport> reports;
};

BmsFrame makeFrame(int cells, int temps)
{
    BmsFrame frame;
    frame.data.packVoltage = 50.0f;
    frame.data.packSOC = 70.0f;
    frame.data.chargeDischargeStatus = 1;
    frame.data.numberOfCells = cells;
    frame.data.numOfTempSensors = temps;
    return frame;
}

class MonitorBatteryUseCaseTest : public ::testing::Test
{
protected:
    std::shared_ptr<FakePublisher> publisher_ = std::make_shared<FakePublisher>();
    MonitorBatteryUseCase use_case_{publisher_, BatteryIdentity{40.0f, "SN"}};
};

}  // namespace

TEST(MonitorBatteryUseCaseCtor, RejectsNullPublisher)
{
    EXPECT_THROW(MonitorBatteryUseCase(nullptr, BatteryIdentity{}), std::invalid_argument);
}

TEST_F(MonitorBatteryUseCaseTest, PublishesOneReportPerFrame)
{
    use_case_.onFrame(makeFrame(16, 4));

    ASSERT_EQ(publisher_->reports.size(), 1u);
    const auto & reading = publisher_->reports.front().reading;
    EXPECT_TRUE(reading.present);
    EXPECT_EQ(reading.charge_state, ChargeState::Charging);
    EXPECT_EQ(reading.serial_number, "SN");
    EXPECT_TRUE(publisher_->reports.front().charging.charging);
}

TEST_F(MonitorBatteryUseCaseTest, TimeoutBeforeAnyFramePublishesEmptyStaleReport)
{
    use_case_.onDataTimeout();

    ASSERT_EQ(publisher_->reports.size(), 1u);
    const auto & report = publisher_->reports.front();
    EXPECT_EQ(report.reading.health, BatteryHealth::WatchdogTimerExpired);
    EXPECT_TRUE(report.reading.cell_voltages.empty());
    EXPECT_TRUE(report.reading.cell_temperatures.empty());
}

TEST_F(MonitorBatteryUseCaseTest, StaleReportKeepsLastArrayShape)
{
    use_case_.onFrame(makeFrame(16, 4));
    use_case_.onDataTimeout();

    ASSERT_EQ(publisher_->reports.size(), 2u);
    const auto & stale = publisher_->reports.back().reading;
    EXPECT_FALSE(stale.present);
    EXPECT_EQ(stale.cell_voltages.size(), 16u);
    EXPECT_EQ(stale.cell_temperatures.size(), 4u);
}

TEST_F(MonitorBatteryUseCaseTest, StaleReportUsesClampedCounts)
{
    use_case_.onFrame(makeFrame(500, 500));
    use_case_.onDataTimeout();

    const auto & stale = publisher_->reports.back().reading;
    EXPECT_EQ(stale.cell_voltages.size(), kBmsMaxCells);
    EXPECT_EQ(stale.cell_temperatures.size(), kBmsMaxTempSensors);
}
