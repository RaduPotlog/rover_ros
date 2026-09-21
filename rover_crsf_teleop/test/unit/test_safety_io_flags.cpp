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

#include "rover_crsf_teleop/domain/safety_io_flags.hpp"

namespace rover_crsf_teleop
{
namespace
{

// The one combination that grants the permit: button pressed, PLC latched, contacts open.
SafetyIoFlags allEngaged()
{
    SafetyIoFlags flags;
    flags.hw_e_stop_user_button = true;
    flags.sw_e_stop_latch_status = true;
    flags.motor_contactor_engaged = false;
    return flags;
}

TEST(SafetyIoFlagsTest, SafeOnlyWhenButtonPressedLatchSetAndContactorOpen)
{
    EXPECT_TRUE(isSafeToCalibrate(allEngaged()));
}

// The case reported from the rover: the software E-Stop (RC switch) has set the latch and the
// contacts are open, but nobody has pressed the physical button. Any Trigger call could clear
// that latch while the operator is mid-sweep, so it must not grant the permit.
TEST(SafetyIoFlagsTest, SoftwareLatchWithoutThePhysicalButtonIsNotEnough)
{
    auto flags = allEngaged();
    flags.hw_e_stop_user_button = false;

    EXPECT_FALSE(isSafeToCalibrate(flags));
}

// Button reads pressed but the PLC has not latched: it has not acted, so the drive is not known
// to be dead.
TEST(SafetyIoFlagsTest, ButtonWithoutTheLatchIsNotEnough)
{
    auto flags = allEngaged();
    flags.sw_e_stop_latch_status = false;

    EXPECT_FALSE(isSafeToCalibrate(flags));
}

// Button pressed and latched, but the contacts did not open - a welded contactor, or the
// drop-out window. Either way the drive may still be live.
TEST(SafetyIoFlagsTest, ContactorStillEngagedIsNotEnough)
{
    auto flags = allEngaged();
    flags.motor_contactor_engaged = true;

    EXPECT_FALSE(isSafeToCalibrate(flags));
}

// A permit AND-s its evidence; no single condition may grant it on its own. This is the regression
// guard against it drifting back to the OR it used to be.
TEST(SafetyIoFlagsTest, NoSingleConditionGrantsThePermitAlone)
{
    SafetyIoFlags only_button;
    only_button.hw_e_stop_user_button = true;
    EXPECT_FALSE(isSafeToCalibrate(only_button));

    SafetyIoFlags only_latch;
    only_latch.sw_e_stop_latch_status = true;
    EXPECT_FALSE(isSafeToCalibrate(only_latch));

    SafetyIoFlags only_contactor_open;
    only_contactor_open.motor_contactor_engaged = false;
    EXPECT_FALSE(isSafeToCalibrate(only_contactor_open));
}

TEST(SafetyIoFlagsTest, TheDefaultsDoNotGrantThePermit)
{
    // rover_twist_mux defaults its equivalent struct to "stop active", because for it that
    // denies motion. Here that would be a silent fail-open, so the defaults are the
    // non-permitting values - including the contactor, which is assumed CLOSED (drive live).
    const SafetyIoFlags defaults;

    EXPECT_FALSE(defaults.hw_e_stop_user_button);
    EXPECT_FALSE(defaults.sw_e_stop_latch_status);
    EXPECT_TRUE(defaults.motor_contactor_engaged);
    EXPECT_FALSE(isSafeToCalibrate(defaults));
}

// Plant state only. Never add a SafetyCommandEcho read-back here: granting a permit on a request
// this system issued itself is a fail-open by construction.
static_assert(sizeof(SafetyIoFlags) == 3 * sizeof(bool),
    "SafetyIoFlags must carry plant state only - see domain/safety_io_flags.hpp before adding a "
    "field, and never add a SafetyCommandEcho read-back here.");

}  // namespace
}  // namespace rover_crsf_teleop
