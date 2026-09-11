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

#include "rover_diag_manager/domain/percentage.hpp"

using namespace rover_diag_manager::domain;  // NOLINT

TEST(RoundTo, RoundsToRequestedDecimals)
{
    EXPECT_FLOAT_EQ(roundTo(12.3456f, 2), 12.35f);
    EXPECT_FLOAT_EQ(roundTo(12.3449f, 2), 12.34f);
    EXPECT_FLOAT_EQ(roundTo(12.0f, 0), 12.0f);
}

TEST(PercentageOf, ComputesPercentage)
{
    const auto result = percentageOf(25.0, 200.0);
    ASSERT_TRUE(result.has_value());
    EXPECT_FLOAT_EQ(*result, 12.5f);
}

TEST(PercentageOf, ReturnsNulloptForZeroTotal)
{
    EXPECT_FALSE(percentageOf(10.0, 0.0).has_value());
}

TEST(PercentageOf, ReturnsNulloptForNegativeTotal)
{
    EXPECT_FALSE(percentageOf(10.0, -5.0).has_value());
}

TEST(MeanUsage, ComputesMeanOfValidCores)
{
    const auto result = meanUsage({10.0f, 20.0f, 30.0f});
    ASSERT_TRUE(result.has_value());
    EXPECT_FLOAT_EQ(*result, 20.0f);
}

TEST(MeanUsage, ReturnsNulloptForEmptyInput)
{
    EXPECT_FALSE(meanUsage({}).has_value());
}

TEST(MeanUsage, ReturnsNulloptWhenAnyCoreOutOfRange)
{
    EXPECT_FALSE(meanUsage({10.0f, 150.0f}).has_value());
    EXPECT_FALSE(meanUsage({-1.0f, 10.0f}).has_value());
}
