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

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"

namespace rover_hardware_interface
{

TEST(PhidgetMotorDriverFailsafeTest, TrippedReturnCodeIsRecognized)
{
    EXPECT_TRUE(PhidgetMotorDriver::isFailsafeTrippedReturnCode(EPHIDGET_FAILSAFE));
}

TEST(PhidgetMotorDriverFailsafeTest, OtherReturnCodesAreNotTreatedAsTripped)
{
    EXPECT_FALSE(PhidgetMotorDriver::isFailsafeTrippedReturnCode(EPHIDGET_OK));
    EXPECT_FALSE(PhidgetMotorDriver::isFailsafeTrippedReturnCode(EPHIDGET_TIMEOUT));
    EXPECT_FALSE(PhidgetMotorDriver::isFailsafeTrippedReturnCode(EPHIDGET_NOTATTACHED));
}

using FailsafeAction = PhidgetMotorDriver::FailsafeAction;

TEST(PhidgetMotorDriverFailsafeTest, NeverEnabledChannelIsEnabled)
{
    EXPECT_EQ(PhidgetMotorDriver::selectFailsafeAction(false, false), FailsafeAction::kEnable);
}

TEST(PhidgetMotorDriverFailsafeTest, AlreadyEnabledHealthyChannelIsOnlyFed)
{
    // Re-enabling an already-enabled failsafe on an open channel is rejected by the SDK.
    EXPECT_EQ(PhidgetMotorDriver::selectFailsafeAction(true, false), FailsafeAction::kFeed);
}

TEST(PhidgetMotorDriverFailsafeTest, TrippedChannelIsReopened)
{
    // A tripped channel rejects both resetFailsafe and enableFailsafe until it is re-opened.
    EXPECT_EQ(
        PhidgetMotorDriver::selectFailsafeAction(true, true), FailsafeAction::kReopenAndEnable);
    EXPECT_EQ(
        PhidgetMotorDriver::selectFailsafeAction(false, true), FailsafeAction::kReopenAndEnable);
}

TEST(PhidgetMotorDriverEncoderTest, CountsConvertToMotorRpm)
{
    // 1024 lines -> 4096 counts/rev; 4096 counts in 1 s is 1 rev/s = 60 RPM.
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::encoderCountsToMotorRpm(4096, 1.0, 1024.0f), 60.0);
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::encoderCountsToMotorRpm(-4096, 0.5, 1024.0f), -120.0);
}

TEST(PhidgetMotorDriverEncoderTest, CountsNotMultipleOfFourAreNotTruncated)
{
    // The old code divided the delta by 4 as an integer and lost the remainder every event:
    // 7 counts read as 1 line (-43 %).
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::encoderCountsToMotorRpm(7, 1.0, 1.0f), 7.0 / 4.0 * 60.0);
}

TEST(PhidgetMotorDriverEncoderTest, SumOfEventsMatchesOneLongInterval)
{
    // Many small odd deltas must integrate to the same distance as one big one (no drift).
    double revolutions = 0.0;
    for (int i = 0; i < 100; ++i) {
        revolutions += PhidgetMotorDriver::encoderCountsToMotorRpm(3, 0.008, 1024.0f) / 60.0 * 0.008;
    }
    EXPECT_NEAR(revolutions, 300.0 / 4096.0, 1e-12);
}

TEST(PhidgetMotorDriverEncoderTest, InvalidIntervalGivesZero)
{
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::encoderCountsToMotorRpm(100, 0.0, 1024.0f), 0.0);
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::encoderCountsToMotorRpm(100, -1.0, 1024.0f), 0.0);
    EXPECT_DOUBLE_EQ(PhidgetMotorDriver::encoderCountsToMotorRpm(100, 1.0, 0.0f), 0.0);
}

TEST(PhidgetMotorDriverEncoderTest, StaleTimeoutSpansSeveralEncoderIntervals)
{
    using std::chrono::milliseconds;
    // DCC1000 minimum encoder interval: a turning wheel must survive a late event.
    EXPECT_EQ(PhidgetMotorDriver::encoderStaleTimeout(50), milliseconds(150));
    EXPECT_EQ(PhidgetMotorDriver::encoderStaleTimeout(200), milliseconds(600));
    EXPECT_GT(PhidgetMotorDriver::encoderStaleTimeout(50), milliseconds(2 * 50));
}

}  // namespace rover_hardware_interface
