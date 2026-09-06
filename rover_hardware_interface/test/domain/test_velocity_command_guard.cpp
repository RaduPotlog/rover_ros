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
#include <limits>
#include <vector>

#include "rover_hardware_interface/domain/velocity_command_guard.hpp"

namespace rover_hardware_interface
{

TEST(VelocityCommandGuardTest, AllZeroIsWithinTolerance)
{
    EXPECT_TRUE(areVelocitiesWithinTolerance({0.0, 0.0, 0.0, 0.0}, 0.01));
}

TEST(VelocityCommandGuardTest, EmptyCommandsAreVacuouslyZero)
{
    EXPECT_TRUE(areVelocitiesWithinTolerance({}, 0.01));
}

TEST(VelocityCommandGuardTest, SmallResidualsWithinToleranceArePermitted)
{
    // The regression this guard exists for: a controller's kinematics leave floating-point
    // residue behind, which the previous std::numeric_limits<double>::epsilon() comparison
    // treated as "still commanding motion" and which made the E-Stop unresettable.
    EXPECT_TRUE(areVelocitiesWithinTolerance({1e-12, -1e-9, 0.0, 5e-3}, 0.01));
}

TEST(VelocityCommandGuardTest, CommandAtExactlyToleranceIsPermitted)
{
    EXPECT_TRUE(areVelocitiesWithinTolerance({0.01, -0.01}, 0.01));
}

TEST(VelocityCommandGuardTest, AnyCommandAboveToleranceIsRejected)
{
    EXPECT_FALSE(areVelocitiesWithinTolerance({0.0, 0.0, 0.0, 0.011}, 0.01));
    EXPECT_FALSE(areVelocitiesWithinTolerance({-0.5, 0.0, 0.0, 0.0}, 0.01));
}

TEST(VelocityCommandGuardTest, NonFiniteCommandsAreRejected)
{
    // Fail-safe: std::abs(NaN) > tolerance is false, so a naive comparison would let an
    // undefined command clear the E-Stop.
    EXPECT_FALSE(
        areVelocitiesWithinTolerance({0.0, std::numeric_limits<double>::quiet_NaN()}, 0.01));
    EXPECT_FALSE(
        areVelocitiesWithinTolerance({std::numeric_limits<double>::infinity(), 0.0}, 0.01));
    EXPECT_FALSE(
        areVelocitiesWithinTolerance({-std::numeric_limits<double>::infinity()}, 0.01));
}

TEST(VelocityCommandGuardTest, DefaultToleranceIsAPhysicalDeadbandNotMachineEpsilon)
{
    EXPECT_GT(kDefaultVelocityCommandZeroTolerance, std::numeric_limits<double>::epsilon());
    EXPECT_LT(kDefaultVelocityCommandZeroTolerance, 0.1);
    EXPECT_TRUE(
        areVelocitiesWithinTolerance({1e-6, -1e-6}, kDefaultVelocityCommandZeroTolerance));
}

}  // namespace rover_hardware_interface
