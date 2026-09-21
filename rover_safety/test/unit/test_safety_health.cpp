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

#include <vector>

#include "rover_safety/domain/safety_health.hpp"

using rover_safety::domain::evaluateSafetyInputs;
using rover_safety::domain::HealthLevel;
using rover_safety::domain::SafetyInput;
using rover_safety::domain::SafetyVerdict;
using rover_safety::domain::verdictHealthLevel;

TEST(SafetyHealth, AllFreshIsOk)
{
    const auto health = evaluateSafetyInputs({
        {"battery", 0.5, 5.0},
        {"safety_status", 100.0, std::nullopt},
    });

    EXPECT_EQ(health.level, HealthLevel::Ok);
    EXPECT_TRUE(health.missing.empty());
    EXPECT_TRUE(health.stale.empty());
}

TEST(SafetyHealth, MissingInputIsWarn)
{
    const auto health = evaluateSafetyInputs({
        {"battery", std::nullopt, 5.0},
        {"safety_status", 1.0, std::nullopt},
    });

    EXPECT_EQ(health.level, HealthLevel::Warn);
    EXPECT_EQ(health.missing, std::vector<std::string>{"battery"});
}

TEST(SafetyHealth, StaleInputIsErrorAndOutranksMissing)
{
    const auto health = evaluateSafetyInputs({
        {"battery", 6.0, 5.0},
        {"system_status", std::nullopt, 5.0},
    });

    EXPECT_EQ(health.level, HealthLevel::Error);
    EXPECT_EQ(health.stale, std::vector<std::string>{"battery"});
    EXPECT_EQ(health.missing, std::vector<std::string>{"system_status"});
}

TEST(SafetyHealth, OnChangeInputNeverGoesStale)
{
    const auto health = evaluateSafetyInputs({{"safety_status", 1e6, std::nullopt}});

    EXPECT_EQ(health.level, HealthLevel::Ok);
}

TEST(SafetyHealth, VerdictLevels)
{
    EXPECT_EQ(verdictHealthLevel(SafetyVerdict::None), HealthLevel::Ok);
    EXPECT_EQ(verdictHealthLevel(SafetyVerdict::TripEStop), HealthLevel::Error);
    EXPECT_EQ(verdictHealthLevel(SafetyVerdict::Shutdown), HealthLevel::Error);
}
