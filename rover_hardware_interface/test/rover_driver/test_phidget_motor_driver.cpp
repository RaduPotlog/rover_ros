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

}  // namespace rover_hardware_interface
