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

#include <stdexcept>

#include "rover_safety/domain/battery_safety_policy.hpp"

using namespace rover_safety::domain;  // NOLINT

namespace
{

const BatteryThresholds kThresholds{50.0, 60.0};

SafetyVerdict verdict(BatteryHealth health, double temperature = 25.0)
{
    return evaluateBatterySafety(health, temperature, kThresholds).verdict;
}

}  // namespace

TEST(BatteryThresholds, RejectsCriticalNotBelowFatal)
{
    EXPECT_THROW(BatteryThresholds(60.0, 60.0), std::invalid_argument);
    EXPECT_THROW(BatteryThresholds(70.0, 60.0), std::invalid_argument);
    EXPECT_NO_THROW(BatteryThresholds(59.9, 60.0));
}

TEST(EvaluateBatterySafety, HealthyBatteryNeedsNoAction)
{
    const auto decision = evaluateBatterySafety(BatteryHealth::Good, 80.0, kThresholds);
    EXPECT_EQ(decision.verdict, SafetyVerdict::None);
    EXPECT_TRUE(decision.reason.empty());
}

TEST(EvaluateBatterySafety, NonCriticalHealthStatesNeedNoAction)
{
    EXPECT_EQ(verdict(BatteryHealth::Unknown, 80.0), SafetyVerdict::None);
    EXPECT_EQ(verdict(BatteryHealth::Cold), SafetyVerdict::None);
    EXPECT_EQ(verdict(BatteryHealth::UnspecFailure), SafetyVerdict::None);
    EXPECT_EQ(verdict(BatteryHealth::SafetyTimerExpire), SafetyVerdict::None);
}

TEST(EvaluateBatterySafety, TripsEStopOnWatchdogDeadOrOvervoltage)
{
    EXPECT_EQ(verdict(BatteryHealth::WatchdogTimerExpire), SafetyVerdict::TripEStop);
    EXPECT_EQ(verdict(BatteryHealth::Dead), SafetyVerdict::TripEStop);
    EXPECT_EQ(verdict(BatteryHealth::Overvoltage), SafetyVerdict::TripEStop);
}

TEST(EvaluateBatterySafety, OverheatEscalatesWithTemperature)
{
    EXPECT_EQ(verdict(BatteryHealth::Overheat, 45.0), SafetyVerdict::None);
    EXPECT_EQ(verdict(BatteryHealth::Overheat, 50.0), SafetyVerdict::None);
    EXPECT_EQ(verdict(BatteryHealth::Overheat, 50.1), SafetyVerdict::TripEStop);
    EXPECT_EQ(verdict(BatteryHealth::Overheat, 60.0), SafetyVerdict::TripEStop);
    EXPECT_EQ(verdict(BatteryHealth::Overheat, 60.1), SafetyVerdict::Shutdown);
}

TEST(EvaluateBatterySafety, HighTemperatureAloneDoesNotTrip)
{
    // The BMS health flag gates the temperature check, as in the original tree.
    EXPECT_EQ(verdict(BatteryHealth::Good, 90.0), SafetyVerdict::None);
}

TEST(EvaluateBatterySafety, ActionsCarryAReason)
{
    EXPECT_EQ(
        evaluateBatterySafety(BatteryHealth::Overheat, 65.0, kThresholds).reason,
        "Fatal battery temperature");
    EXPECT_EQ(
        evaluateBatterySafety(BatteryHealth::Dead, 25.0, kThresholds).reason, "Battery dead");
}
