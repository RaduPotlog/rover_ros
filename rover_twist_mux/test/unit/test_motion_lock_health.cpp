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

#include <optional>

#include "rover_twist_mux/domain/motion_lock_health.hpp"

using namespace rover_twist_mux::domain;  // NOLINT

namespace
{

constexpr double kTimeout = 1.0;

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

// Both messages the same age: the single pre-combined age these cases were written against.
SafetyStateAges bothAged(const double age_s) { return SafetyStateAges{age_s, age_s}; }


// The link going down is invisible to a staleness check: the hardware interface keeps publishing
// at 20 Hz, it just has nothing fresh to put in the messages. Without this the lock would happily
// unlock on last-known-good values from a PLC that stopped answering.
TEST(MotionLockHealth, UnhealthySafetyLinkIsErrorAndLockedEvenWhenAllClear)
{
    SafetyIoFlags flags;
    flags.hw_e_stop_user_button = false;
    flags.sw_e_stop_user_button = false;
    flags.sw_e_stop_motor_driver_fault = false;
    flags.sw_e_stop_latch_status = false;
    flags.motor_contactor_engaged = true;

    const auto health = evaluateMotionLockHealth(
        flags, bothAged(0.0), 1.0, MotionLockPolicy{}, false);

    EXPECT_TRUE(health.locked);
    EXPECT_EQ(health.level, HealthLevel::Error);
    ASSERT_FALSE(health.reasons.empty());
    EXPECT_EQ(health.reasons.back(), MotionInhibitReason::SafetyLinkUnhealthy);
}

TEST(MotionLockHealth, HealthySafetyLinkWithNoStopsPermitsMotion)
{
    SafetyIoFlags flags;
    flags.hw_e_stop_user_button = false;
    flags.sw_e_stop_user_button = false;
    flags.sw_e_stop_motor_driver_fault = false;
    flags.sw_e_stop_latch_status = false;
    flags.motor_contactor_engaged = true;

    const auto health = evaluateMotionLockHealth(
        flags, bothAged(0.0), 1.0, MotionLockPolicy{}, true);

    EXPECT_FALSE(health.locked);
    EXPECT_EQ(health.level, HealthLevel::Ok);
}

}  // namespace

TEST(MotionLockHealth, NoSafetyStateIsErrorAndLocked)
{
    const auto health = evaluateMotionLockHealth(
        std::nullopt, bothAged(0.0), kTimeout, MotionLockPolicy{});

    EXPECT_EQ(health.level, HealthLevel::Error);
    EXPECT_TRUE(health.locked);
    EXPECT_TRUE(health.reasons.empty());
}

TEST(MotionLockHealth, StaleSafetyStateIsErrorAndLockedEvenWhenAllClear)
{
    const auto health = evaluateMotionLockHealth(
        allClear(), bothAged(1.5), kTimeout, MotionLockPolicy{});

    EXPECT_EQ(health.level, HealthLevel::Error);
    EXPECT_TRUE(health.locked);
    EXPECT_NE(health.message.find("stale"), std::string::npos);
}

TEST(MotionLockHealth, AllClearIsOkAndUnlocked)
{
    const auto health = evaluateMotionLockHealth(
        allClear(), bothAged(0.1), kTimeout, MotionLockPolicy{});

    EXPECT_EQ(health.level, HealthLevel::Ok);
    EXPECT_FALSE(health.locked);
}

TEST(MotionLockHealth, ActiveStopIsWarnWithReason)
{
    auto flags = allClear();
    flags.hw_e_stop_user_button = true;

    const auto health = evaluateMotionLockHealth(
        flags, bothAged(0.1), kTimeout, MotionLockPolicy{});

    EXPECT_EQ(health.level, HealthLevel::Warn);
    EXPECT_TRUE(health.locked);
    ASSERT_EQ(health.reasons.size(), 1u);
    EXPECT_EQ(health.reasons.front(), MotionInhibitReason::HwEStopUserButton);
    EXPECT_NE(health.message.find(toString(MotionInhibitReason::HwEStopUserButton)), std::string::npos);
}

TEST(MotionLockHealth, LockedAgreesWithPolicyWhenFresh)
{
    // The published lock comes from `locked`; it must never diverge from the policy decision.
    auto flags = allClear();
    flags.sw_e_stop_motor_driver_fault = true;

    const MotionLockPolicy policy;
    const auto health = evaluateMotionLockHealth(flags, bothAged(0.0), kTimeout, policy);

    EXPECT_EQ(health.locked, isMotionInhibited(flags, policy));
}

// Staleness is judged on the OLDER of the two ages: one topic going quiet must lock the rover
// even while the other keeps arriving. The two cases below catch a rule that took the newer age
// or averaged them; the "1.50" in the message proves which age was compared.
TEST(MotionLockHealth, StaleCommandEchoLocksEvenWhileSafetyStatusIsFresh)
{
    const auto health = evaluateMotionLockHealth(
        allClear(), SafetyStateAges{0.1, 1.5}, kTimeout, MotionLockPolicy{});

    EXPECT_EQ(health.level, HealthLevel::Error);
    EXPECT_TRUE(health.locked);
    EXPECT_TRUE(health.reasons.empty());
    EXPECT_NE(health.message.find("stale (1.50 s"), std::string::npos) << health.message;
}

TEST(MotionLockHealth, StaleSafetyStatusLocksEvenWhileCommandEchoIsFresh)
{
    const auto health = evaluateMotionLockHealth(
        allClear(), SafetyStateAges{1.5, 0.1}, kTimeout, MotionLockPolicy{});

    EXPECT_EQ(health.level, HealthLevel::Error);
    EXPECT_TRUE(health.locked);
    EXPECT_TRUE(health.reasons.empty());
    EXPECT_NE(health.message.find("stale (1.50 s"), std::string::npos) << health.message;
}

// Both fresh but unequal: their sum is past the timeout, so this catches a rule that added them.
TEST(MotionLockHealth, UnequalFreshAgesPermitMotion)
{
    const auto health = evaluateMotionLockHealth(
        allClear(), SafetyStateAges{0.2, 0.9}, kTimeout, MotionLockPolicy{});

    EXPECT_EQ(health.level, HealthLevel::Ok);
    EXPECT_FALSE(health.locked);
}

// The comparison is strict: an older age exactly at the timeout is still fresh.
TEST(MotionLockHealth, OlderAgeExactlyAtTheTimeoutIsStillFresh)
{
    const auto health = evaluateMotionLockHealth(
        allClear(), SafetyStateAges{0.0, kTimeout}, kTimeout, MotionLockPolicy{});

    EXPECT_EQ(health.level, HealthLevel::Ok);
    EXPECT_FALSE(health.locked);
}

// Until both topics have arrived there is nothing to age, so the ages must not decide the message.
TEST(MotionLockHealth, AgesAreIgnoredUntilBothTopicsHaveArrived)
{
    const auto health = evaluateMotionLockHealth(
        std::nullopt, SafetyStateAges{5.0, 5.0}, kTimeout, MotionLockPolicy{});

    EXPECT_EQ(health.level, HealthLevel::Error);
    EXPECT_TRUE(health.locked);
    EXPECT_NE(health.message.find("No safety state"), std::string::npos) << health.message;
    EXPECT_EQ(health.message.find("stale"), std::string::npos) << health.message;
}
