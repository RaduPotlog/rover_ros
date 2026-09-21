// Copyright 2026 Mechatronics Academy
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
//
// Unit tests for ContactorMonitor. Time is injected, so none of these wait in real time.

#include <gtest/gtest.h>

#include <chrono>

#include "rover_hardware_interface/domain/contactor_monitor.hpp"

namespace rover_hardware_interface
{
namespace test
{

using namespace std::chrono_literals;

class ContactorMonitorTest : public ::testing::Test
{
protected:
    ContactorMonitor monitor {500ms};
    std::chrono::steady_clock::time_point t0 {};
};

TEST_F(ContactorMonitorTest, HealthyWhenLatchClearAndContactorEngaged)
{
    EXPECT_EQ(monitor.update(false, true, t0), ContactorFault::kNone);
    EXPECT_FALSE(monitor.isWeldedFaultLatched());
    EXPECT_FALSE(monitor.isFailedOpen());
}

TEST_F(ContactorMonitorTest, HealthyWhenLatchAssertedAndContactorOpen)
{
    EXPECT_EQ(monitor.update(true, false, t0), ContactorFault::kNone);
    EXPECT_FALSE(monitor.isWeldedFaultLatched());
}

// The whole point: an ordinary E-Stop must not raise a fault during the moments between the
// relay latching and the contacts physically parting.
TEST_F(ContactorMonitorTest, ToleratesDisagreementWithinDropOutTime)
{
    EXPECT_EQ(monitor.update(true, true, t0), ContactorFault::kNone);
    EXPECT_EQ(monitor.update(true, true, t0 + 100ms), ContactorFault::kNone);
    EXPECT_EQ(monitor.update(true, true, t0 + 499ms), ContactorFault::kNone);
    EXPECT_FALSE(monitor.isWeldedFaultLatched());
}

TEST_F(ContactorMonitorTest, RaisesWeldedFaultOnceDisagreementOutlastsDropOutTime)
{
    ASSERT_EQ(monitor.update(true, true, t0), ContactorFault::kNone);

    EXPECT_EQ(monitor.update(true, true, t0 + 500ms), ContactorFault::kWeldedSuspected);
    EXPECT_TRUE(monitor.isWeldedFaultLatched());
}

TEST_F(ContactorMonitorTest, WeldedFaultStaysLatchedAfterTheLatchClears)
{
    ASSERT_EQ(monitor.update(true, true, t0), ContactorFault::kNone);
    ASSERT_EQ(monitor.update(true, true, t0 + 600ms), ContactorFault::kWeldedSuspected);

    // Contactor now looks healthy again. The fault must not evaporate: a contactor that failed
    // to open once needs a human to look at it.
    EXPECT_EQ(monitor.update(false, true, t0 + 700ms), ContactorFault::kWeldedSuspected);
    EXPECT_TRUE(monitor.isWeldedFaultLatched());
}

TEST_F(ContactorMonitorTest, ResetClearsTheLatchedWeldedFault)
{
    ASSERT_EQ(monitor.update(true, true, t0), ContactorFault::kNone);
    ASSERT_EQ(monitor.update(true, true, t0 + 600ms), ContactorFault::kWeldedSuspected);

    monitor.reset();

    EXPECT_FALSE(monitor.isWeldedFaultLatched());
    EXPECT_EQ(monitor.update(false, true, t0 + 700ms), ContactorFault::kNone);
}

// A disagreement that clears and later recurs must start its grace period over, rather than
// carrying forward the earlier elapsed time and tripping early.
TEST_F(ContactorMonitorTest, DisagreementTimerRestartsAfterAgreement)
{
    ASSERT_EQ(monitor.update(true, true, t0), ContactorFault::kNone);
    ASSERT_EQ(monitor.update(true, true, t0 + 400ms), ContactorFault::kNone);

    // Contacts part - agreement restored.
    ASSERT_EQ(monitor.update(true, false, t0 + 450ms), ContactorFault::kNone);

    // New disagreement at +500ms gets a full fresh tolerance window.
    EXPECT_EQ(monitor.update(true, true, t0 + 500ms), ContactorFault::kNone);
    EXPECT_EQ(monitor.update(true, true, t0 + 900ms), ContactorFault::kNone);
    EXPECT_EQ(monitor.update(true, true, t0 + 1000ms), ContactorFault::kWeldedSuspected);
}

TEST_F(ContactorMonitorTest, ReportsFailedOpenWithoutLatching)
{
    EXPECT_EQ(monitor.update(false, false, t0), ContactorFault::kFailedOpen);
    EXPECT_TRUE(monitor.isFailedOpen());
    EXPECT_FALSE(monitor.isWeldedFaultLatched());

    // Clears on its own once the contactor engages - it is a report, not a fault.
    EXPECT_EQ(monitor.update(false, true, t0 + 10ms), ContactorFault::kNone);
    EXPECT_FALSE(monitor.isFailedOpen());
}

TEST_F(ContactorMonitorTest, WeldedFaultOutranksFailedOpenInTheReturnedVerdict)
{
    ASSERT_EQ(monitor.update(true, true, t0), ContactorFault::kNone);
    ASSERT_EQ(monitor.update(true, true, t0 + 600ms), ContactorFault::kWeldedSuspected);

    EXPECT_EQ(monitor.update(false, false, t0 + 700ms), ContactorFault::kWeldedSuspected);
}

TEST_F(ContactorMonitorTest, ReportsDisagreementDuration)
{
    EXPECT_EQ(monitor.disagreementDuration(t0), 0ms);

    ASSERT_EQ(monitor.update(true, true, t0), ContactorFault::kNone);
    EXPECT_EQ(monitor.disagreementDuration(t0 + 250ms), 250ms);

    ASSERT_EQ(monitor.update(true, false, t0 + 300ms), ContactorFault::kNone);
    EXPECT_EQ(monitor.disagreementDuration(t0 + 300ms), 0ms);
}

}  // namespace test
}  // namespace rover_hardware_interface
