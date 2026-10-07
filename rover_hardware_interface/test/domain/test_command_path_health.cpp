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

#include <cstdint>

#include "rover_hardware_interface/domain/command_path_health.hpp"

namespace rover_hardware_interface
{

namespace
{
constexpr std::int64_t kS = 1000000000;  // ns per second
}

TEST(ErrorTrendTest, StartsQuiet)
{
    ErrorTrend t;

    EXPECT_EQ(t.total(), 0u);
    EXPECT_FALSE(t.increasedWithin(10 * kS, 3 * kS));
    EXPECT_LT(t.secondsSinceIncrease(10 * kS), 0.0);
}

TEST(ErrorTrendTest, NoErrorsEverStaysQuietEvenAfterManyUpdates)
{
    ErrorTrend t;

    for (int i = 0; i < 5; ++i) {
        t.update(0, i * kS);
    }

    EXPECT_FALSE(t.increasedWithin(5 * kS, 3 * kS));
}

TEST(ErrorTrendTest, FirstNonZeroTotalCountsAsAnIncrease)
{
    // Errors that predate the first look are still news.
    ErrorTrend t;
    t.update(7814, 100 * kS);

    EXPECT_TRUE(t.increasedWithin(100 * kS, 3 * kS));
    EXPECT_DOUBLE_EQ(t.secondsSinceIncrease(102 * kS), 2.0);
}

TEST(ErrorTrendTest, GrowingCounterStaysIncreasing)
{
    ErrorTrend t;

    t.update(10, 0 * kS);
    t.update(35, 1 * kS);
    t.update(60, 2 * kS);

    EXPECT_TRUE(t.increasedWithin(2 * kS, 3 * kS));
    EXPECT_DOUBLE_EQ(t.secondsSinceIncrease(2 * kS), 0.0);
}

TEST(ErrorTrendTest, CounterThatStoppedGrowingAgesOutOfTheWindow)
{
    ErrorTrend t;

    t.update(10, 0 * kS);
    t.update(60, 1 * kS);   // last increase
    t.update(60, 2 * kS);
    t.update(60, 3 * kS);

    EXPECT_TRUE(t.increasedWithin(3 * kS, 3 * kS));   // 2 s ago: still "recent"
    t.update(60, 6 * kS);
    EXPECT_FALSE(t.increasedWithin(6 * kS, 3 * kS));  // 5 s ago: over
    EXPECT_DOUBLE_EQ(t.secondsSinceIncrease(6 * kS), 5.0);
    EXPECT_EQ(t.total(), 60u);
}

TEST(CommandPathHealthTest, QuietPathIsOk)
{
    const auto v = evaluateCommandPathHealth({});

    EXPECT_EQ(v.level, HealthLevel::kOk);
}

TEST(CommandPathHealthTest, ErrorsStillIncreasingIsAnError)
{
    CommandPathHealthInput in;
    in.command_errors_increasing = true;
    in.command_errors_total = 7814;
    in.seconds_since_command_error = 0.0;

    const auto v = evaluateCommandPathHealth(in);

    EXPECT_EQ(v.level, HealthLevel::kError);
    EXPECT_NE(v.message.find("failing"), std::string::npos);
}

TEST(CommandPathHealthTest, PastErrorsThatStoppedAreOnlyAWarning)
{
    CommandPathHealthInput in;
    in.command_errors_total = 12;
    in.seconds_since_command_error = 42.0;

    const auto v = evaluateCommandPathHealth(in);

    EXPECT_EQ(v.level, HealthLevel::kWarn);
    EXPECT_NE(v.message.find("42"), std::string::npos);
}

TEST(CommandPathHealthTest, ErrorsTakePrecedenceOverEverythingElse)
{
    CommandPathHealthInput in;
    in.command_errors_increasing = true;
    in.command_errors_total = 1;
    in.dropped_no_driver = 3;
    in.write_exceptions = 2;

    EXPECT_EQ(evaluateCommandPathHealth(in).level, HealthLevel::kError);
}

TEST(CommandPathHealthTest, MissingDriverOrThrowingWriteIsAWarning)
{
    CommandPathHealthInput a;
    a.dropped_no_driver = 1;
    EXPECT_EQ(evaluateCommandPathHealth(a).level, HealthLevel::kWarn);

    CommandPathHealthInput b;
    b.write_exceptions = 1;
    EXPECT_EQ(evaluateCommandPathHealth(b).level, HealthLevel::kWarn);
}

}  // namespace rover_hardware_interface
