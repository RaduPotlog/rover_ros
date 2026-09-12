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

#include "rover_twist_mux/domain/motion_lock_policy.hpp"

using namespace rover_twist_mux::domain;  // NOLINT

namespace
{

/** @brief Every stop clear and the contactor engaged - the only state that permits motion. */
SafetyIoFlags allClear()
{
    SafetyIoFlags flags;

    flags.hw_e_stop_user_button = false;
    flags.sw_e_stop_user_button = false;
    flags.sw_e_stop_motor_driver_fault = false;
    flags.sw_e_stop_latch_status = false;
    flags.motor_contactor_engaged = true;

    return flags;
}

}  // namespace

TEST(MotionLockPolicy, DefaultConstructedFlagsDenyMotion)
{
    // The fail-safe default: a value nobody populated must not open the command path.
    EXPECT_TRUE(isMotionInhibited(SafetyIoFlags{}, MotionLockPolicy{}));
}

TEST(MotionLockPolicy, AllClearPermitsMotion)
{
    EXPECT_FALSE(isMotionInhibited(allClear(), MotionLockPolicy{}));
}

TEST(MotionLockPolicy, HardwareEStopInhibits)
{
    auto flags = allClear();
    flags.hw_e_stop_user_button = true;

    EXPECT_TRUE(isMotionInhibited(flags, MotionLockPolicy{}));
}

TEST(MotionLockPolicy, SoftwareEStopInhibits)
{
    auto flags = allClear();
    flags.sw_e_stop_user_button = true;

    EXPECT_TRUE(isMotionInhibited(flags, MotionLockPolicy{}));
}

TEST(MotionLockPolicy, MotorDriverFaultInhibits)
{
    auto flags = allClear();
    flags.sw_e_stop_motor_driver_fault = true;

    EXPECT_TRUE(isMotionInhibited(flags, MotionLockPolicy{}));
}

TEST(MotionLockPolicy, LatchHeldInhibits)
{
    auto flags = allClear();
    flags.sw_e_stop_latch_status = true;

    EXPECT_TRUE(isMotionInhibited(flags, MotionLockPolicy{}));
}

TEST(MotionLockPolicy, DisabledPinIsIgnored)
{
    auto flags = allClear();
    flags.hw_e_stop_user_button = true;

    MotionLockPolicy policy;
    policy.use_hw_e_stop_user_button = false;

    EXPECT_FALSE(isMotionInhibited(flags, policy));
}

TEST(MotionLockPolicy, ContactorIsIgnoredByDefault)
{
    // Inverted sense, and off by default: an open contactor alone must not lock.
    auto flags = allClear();
    flags.motor_contactor_engaged = false;

    EXPECT_FALSE(isMotionInhibited(flags, MotionLockPolicy{}));
}

TEST(MotionLockPolicy, ContactorInhibitsWhenRequiredAndDisengaged)
{
    auto flags = allClear();
    flags.motor_contactor_engaged = false;

    MotionLockPolicy policy;
    policy.require_motor_contactor_engaged = true;

    EXPECT_TRUE(isMotionInhibited(flags, policy));
}

TEST(MotionLockPolicy, ContactorPermitsWhenRequiredAndEngaged)
{
    MotionLockPolicy policy;
    policy.require_motor_contactor_engaged = true;

    EXPECT_FALSE(isMotionInhibited(allClear(), policy));
}

TEST(MotionLockPolicy, NoPinEnabledNeverInhibits)
{
    MotionLockPolicy policy;
    policy.use_hw_e_stop_user_button = false;
    policy.use_sw_e_stop_user_button = false;
    policy.use_sw_e_stop_motor_driver_fault = false;
    policy.use_sw_e_stop_latch_status = false;

    EXPECT_FALSE(isMotionInhibited(SafetyIoFlags{}, policy));
}

TEST(MotionLockPolicy, AnySingleActiveStopIsEnough)
{
    // Independence: each condition inhibits on its own, none shadows another.
    for (int pin = 0; pin < 4; ++pin) {
        auto flags = allClear();

        switch (pin) {
            case 0: flags.hw_e_stop_user_button = true; break;
            case 1: flags.sw_e_stop_user_button = true; break;
            case 2: flags.sw_e_stop_motor_driver_fault = true; break;
            case 3: flags.sw_e_stop_latch_status = true; break;
            default: FAIL() << "unreachable";
        }

        EXPECT_TRUE(isMotionInhibited(flags, MotionLockPolicy{})) << "pin index " << pin;
    }
}

TEST(MotionLockPolicy, CpuWatchdogHeartbeatIsNotAStopCondition)
{
    // Regression: GpioState.gpio_pin_cpu_wdg_heartbeat is a ~1 Hz square wave the safety
    // controller drives to feed the relay's watchdog. It used to be wired in as an active-high
    // stop, which made the published motion lock oscillate with it. SafetyIoFlags deliberately has
    // no member for it, so this is a compile-time guarantee - the test pins the intent: with every
    // real stop clear, motion stays permitted no matter what the heartbeat is doing.
    EXPECT_FALSE(isMotionInhibited(allClear(), MotionLockPolicy{}));

    MotionLockPolicy all_enabled;
    all_enabled.require_motor_contactor_engaged = true;

    EXPECT_FALSE(isMotionInhibited(allClear(), all_enabled));
}
