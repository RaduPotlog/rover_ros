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

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rover_gps_heading/domain/geo_math.hpp"

using namespace rover_gps_heading::domain;  // NOLINT

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(GeoMathTest, NorthOffsetOfOneMicroDegreeLatitude)
{
    const EnuOffset offset = enuOffset(45.0, 25.0, 45.00001, 25.0);
    EXPECT_NEAR(offset.east_m, 0.0, 1e-9);
    EXPECT_NEAR(offset.north_m, 1.1132, 1e-3);
}

TEST(GeoMathTest, EastOffsetShrinksWithLatitude)
{
    const EnuOffset equator = enuOffset(0.0, 25.0, 0.0, 25.00001);
    const EnuOffset sixty = enuOffset(60.0, 25.0, 60.0, 25.00001);
    EXPECT_NEAR(equator.east_m, 1.1132, 1e-3);
    EXPECT_NEAR(sixty.east_m, equator.east_m * 0.5, 1e-3);
    EXPECT_NEAR(sixty.north_m, 0.0, 1e-9);
}

TEST(GeoMathTest, AntimeridianCrossingStaysShort)
{
    const EnuOffset offset = enuOffset(0.0, 179.99999, 0.0, -179.99999);
    EXPECT_NEAR(offset.east_m, 2.2264, 1e-3);
}

TEST(GeoMathTest, WrapAngle)
{
    EXPECT_NEAR(wrapAngle(0.5), 0.5, 1e-12);
    EXPECT_NEAR(wrapAngle(2.0 * kPi + 0.5), 0.5, 1e-12);
    EXPECT_NEAR(wrapAngle(-2.0 * kPi - 0.5), -0.5, 1e-12);
    EXPECT_NEAR(std::abs(wrapAngle(kPi)), kPi, 1e-12);
}

TEST(GeoMathTest, CircularStatsAcrossWrap)
{
    const CircularStats stats = circularStats({kPi - 0.05, -kPi + 0.05});
    EXPECT_NEAR(std::abs(stats.mean_rad), kPi, 1e-9);
    EXPECT_NEAR(stats.std_rad, 0.05, 1e-3);
}

TEST(GeoMathTest, CircularStatsOfIdenticalAnglesHasZeroStd)
{
    const CircularStats stats = circularStats({1.0, 1.0, 1.0});
    EXPECT_NEAR(stats.mean_rad, 1.0, 1e-12);
    EXPECT_NEAR(stats.std_rad, 0.0, 1e-6);
}
