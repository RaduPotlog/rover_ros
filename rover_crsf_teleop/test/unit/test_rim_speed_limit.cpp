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

#include <cmath>

#include "rover_crsf_teleop/domain/rim_speed_limit.hpp"

namespace rover_crsf_teleop
{
namespace
{

// The A1: wheel_separation 0.62602 x wheel_separation_multiplier 1.63, halved.
constexpr double kHalfTrack = 0.62602 * 1.63 / 2.0;
constexpr double kBudget = 1.7;

double rimSpeed(const VelocityCommand & command)
{
    return std::abs(command.linear_x) + std::abs(command.angular_z) * kHalfTrack;
}

}  // namespace

TEST(RimSpeedLimitTest, WithinBudgetIsUnchanged)
{
    const VelocityCommand in{0.8, 0.5};
    const auto out = limitRimSpeed(in, kBudget, kHalfTrack);
    EXPECT_DOUBLE_EQ(out.linear_x, 0.8);
    EXPECT_DOUBLE_EQ(out.angular_z, 0.5);
}

TEST(RimSpeedLimitTest, OverBudgetScalesToBudgetAndKeepsCurvature)
{
    // The log that motivated this: 1.5 m/s + 2.1 rad/s put the outer wheel at 2.57 m/s.
    const VelocityCommand in{1.5, 2.1};
    const auto out = limitRimSpeed(in, kBudget, kHalfTrack);
    EXPECT_NEAR(rimSpeed(out), kBudget, 1e-12);
    EXPECT_NEAR(out.angular_z / out.linear_x, 2.1 / 1.5, 1e-12);
}

TEST(RimSpeedLimitTest, SignsArePreserved)
{
    const VelocityCommand in{-1.5, 2.1};
    const auto out = limitRimSpeed(in, kBudget, kHalfTrack);
    EXPECT_LT(out.linear_x, 0.0);
    EXPECT_GT(out.angular_z, 0.0);
    EXPECT_NEAR(rimSpeed(out), kBudget, 1e-12);
}

TEST(RimSpeedLimitTest, PureStraightAndPureSpin)
{
    const auto straight = limitRimSpeed(VelocityCommand{2.0, 0.0}, kBudget, kHalfTrack);
    EXPECT_DOUBLE_EQ(straight.linear_x, kBudget);
    EXPECT_DOUBLE_EQ(straight.angular_z, 0.0);

    const auto spin = limitRimSpeed(VelocityCommand{0.0, -5.0}, kBudget, kHalfTrack);
    EXPECT_DOUBLE_EQ(spin.linear_x, 0.0);
    EXPECT_NEAR(spin.angular_z, -kBudget / kHalfTrack, 1e-12);
}

TEST(RimSpeedLimitTest, ZeroStaysExactlyZero)
{
    const auto out = limitRimSpeed(VelocityCommand{}, kBudget, kHalfTrack);
    EXPECT_TRUE(out.isZero());
}

TEST(RimSpeedLimitTest, NonPositiveBudgetOrTrackDisablesTheLimit)
{
    const VelocityCommand in{1.5, 2.1};
    for (const auto & out : {limitRimSpeed(in, 0.0, kHalfTrack), limitRimSpeed(in, -1.0, kHalfTrack),
                             limitRimSpeed(in, kBudget, 0.0)}) {
        EXPECT_DOUBLE_EQ(out.linear_x, 1.5);
        EXPECT_DOUBLE_EQ(out.angular_z, 2.1);
    }
}

}  // namespace rover_crsf_teleop
