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

#include "rover_crfs_teleop/domain/stick_mapping.hpp"

namespace rover_crfs_telop
{
namespace
{

// The production linear.x mapping: raw CRSF endpoints, +-2 m/s, no inversion.
AxisMapping linearMapping(const int deadband = 0)
{
    AxisMapping mapping;
    mapping.out_min = -2.0;
    mapping.out_max = 2.0;
    mapping.deadband_counts = deadband;
    return mapping;
}

}  // namespace

TEST(StickMappingTest, CentredStickIsExactlyZero)
{
    // The regression this whole mapping exists for. The previous [0, 2000] -> [-2, 2] map put the
    // CRSF midpoint (992) at -0.016 m/s, so the rover was permanently commanded to creep and the
    // hardware interface refused every E-Stop reset ("velocity commands are not zero").
    EXPECT_EQ(mapAxis(kDefaultCrsfChannelMid, linearMapping()), 0.0);
}

TEST(StickMappingTest, DefaultDeadbandAbsorbsTheMeasuredRestingOffsets)
{
    // Regression guard for the second half of the "Can't reset User E-Stop" bug. The mapping was
    // already centred on 992, but the deadband was 8 counts while the A1's transmitter rests at
    // ch3=1004 and ch1=987 (measured off /rc/channels). The 12-count linear offset leaked through
    // as 0.0099 m/s -> 0.0597 rad/s at the wheels, which is ~6x the hardware interface's E-Stop
    // reset tolerance, so the E-Stop stayed unresettable from Foxglove and RC alike.
    AxisMapping linear;
    linear.out_min = -2.0;
    linear.out_max = 2.0;
    linear.deadband_counts = kDefaultChannelDeadband;

    AxisMapping angular;
    angular.out_min = -5.0;
    angular.out_max = 5.0;
    angular.deadband_counts = kDefaultChannelDeadband;
    angular.invert = true;

    EXPECT_EQ(mapAxis(1004, linear), 0.0);
    EXPECT_EQ(mapAxis(987, angular), 0.0);

    // ...with margin for drift on both sides, but still far short of a deliberate deflection.
    EXPECT_EQ(mapAxis(kDefaultCrsfChannelMid + 20, linear), 0.0);
    EXPECT_EQ(mapAxis(kDefaultCrsfChannelMid - 20, linear), 0.0);
    EXPECT_NE(mapAxis(kDefaultCrsfChannelMid + 100, linear), 0.0);
}

TEST(StickMappingTest, DeadbandMapsToExactlyZero)
{
    const auto mapping = linearMapping(8);

    EXPECT_EQ(mapAxis(kDefaultCrsfChannelMid, mapping), 0.0);
    EXPECT_EQ(mapAxis(kDefaultCrsfChannelMid + 8, mapping), 0.0);
    EXPECT_EQ(mapAxis(kDefaultCrsfChannelMid - 8, mapping), 0.0);

    EXPECT_NE(mapAxis(kDefaultCrsfChannelMid + 9, mapping), 0.0);
    EXPECT_NE(mapAxis(kDefaultCrsfChannelMid - 9, mapping), 0.0);
}

TEST(StickMappingTest, EndpointsReachTheOutputLimits)
{
    const auto mapping = linearMapping(8);

    EXPECT_DOUBLE_EQ(mapAxis(kDefaultCrsfChannelMax, mapping), 2.0);
    EXPECT_DOUBLE_EQ(mapAxis(kDefaultCrsfChannelMin, mapping), -2.0);
}

TEST(StickMappingTest, OutOfRangeValuesAreClamped)
{
    const auto mapping = linearMapping();

    EXPECT_DOUBLE_EQ(mapAxis(5000, mapping), 2.0);
    EXPECT_DOUBLE_EQ(mapAxis(-5000, mapping), -2.0);
    EXPECT_DOUBLE_EQ(mapAxis(0, mapping), -2.0);
}

TEST(StickMappingTest, DeflectionIsMonotonicAndSignCorrect)
{
    const auto mapping = linearMapping();

    EXPECT_GT(mapAxis(kDefaultCrsfChannelMid + 100, mapping), 0.0);
    EXPECT_LT(mapAxis(kDefaultCrsfChannelMid - 100, mapping), 0.0);
    EXPECT_GT(
        mapAxis(kDefaultCrsfChannelMid + 400, mapping),
        mapAxis(kDefaultCrsfChannelMid + 100, mapping));
}

TEST(StickMappingTest, InvertFlipsTheAxisButKeepsTheCentreAtZero)
{
    AxisMapping mapping;
    mapping.out_min = -5.0;
    mapping.out_max = 5.0;
    mapping.invert = true;

    EXPECT_EQ(mapAxis(kDefaultCrsfChannelMid, mapping), 0.0);
    EXPECT_DOUBLE_EQ(mapAxis(kDefaultCrsfChannelMax, mapping), -5.0);
    EXPECT_DOUBLE_EQ(mapAxis(kDefaultCrsfChannelMin, mapping), 5.0);
}

TEST(StickMappingTest, AsymmetricRangeStillCentresAtZeroAndReachesBothLimits)
{
    // A trimmed transmitter whose midpoint is not halfway between the endpoints.
    AxisMapping mapping;
    mapping.in_min = 172;
    mapping.in_mid = 900;
    mapping.in_max = 1811;
    mapping.out_min = -2.0;
    mapping.out_max = 2.0;

    EXPECT_EQ(mapAxis(900, mapping), 0.0);
    EXPECT_DOUBLE_EQ(mapAxis(1811, mapping), 2.0);
    EXPECT_DOUBLE_EQ(mapAxis(172, mapping), -2.0);
}

TEST(StickMappingTest, AsymmetricOutputLimitsAreRespected)
{
    AxisMapping mapping;
    mapping.out_min = -1.0;
    mapping.out_max = 3.0;

    EXPECT_DOUBLE_EQ(mapAxis(kDefaultCrsfChannelMax, mapping), 3.0);
    EXPECT_DOUBLE_EQ(mapAxis(kDefaultCrsfChannelMin, mapping), -1.0);
}

TEST(StickMappingTest, DegenerateMappingReturnsZeroInsteadOfDividingByZero)
{
    AxisMapping mapping;
    mapping.deadband_counts = 10000;  // wider than the whole throw

    EXPECT_EQ(mapAxis(kDefaultCrsfChannelMax, mapping), 0.0);
    EXPECT_EQ(mapAxis(kDefaultCrsfChannelMin, mapping), 0.0);
}

}  // namespace rover_crfs_telop
