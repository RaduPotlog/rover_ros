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

#include "rover_crfs_teleop/domain/switch_debouncer.hpp"

namespace rover_crfs_teleop
{
namespace
{

constexpr int kThreshold = 500;
constexpr int kLow = 172;
constexpr int kHigh = 1811;

}  // namespace

TEST(SwitchDebouncerTest, NeverEmitsDuringSettle)
{
    SwitchDebouncer debouncer(kThreshold, 3);

    EXPECT_FALSE(debouncer.update(kLow).has_value());
    EXPECT_FALSE(debouncer.update(kHigh).has_value());
    EXPECT_FALSE(debouncer.update(kLow).has_value());
}

TEST(SwitchDebouncerTest, RestingPositionIsNotAnEdge)
{
    // The startup regression: a switch resting low must not fire an E-Stop set on the first
    // frame after settling.
    SwitchDebouncer debouncer(kThreshold, 2);
    debouncer.update(kLow);
    debouncer.update(kLow);

    EXPECT_FALSE(debouncer.update(kLow).has_value());
}

TEST(SwitchDebouncerTest, ZeroSettleFramesStillLatchesTheFirstFrame)
{
    SwitchDebouncer debouncer(kThreshold, 0);

    EXPECT_FALSE(debouncer.update(kHigh).has_value());
    EXPECT_EQ(debouncer.update(kLow), SwitchPosition::kLow);
}

TEST(SwitchDebouncerTest, EachChangeEmitsExactlyOnce)
{
    SwitchDebouncer debouncer(kThreshold, 0);
    debouncer.update(kHigh);

    EXPECT_EQ(debouncer.update(kLow), SwitchPosition::kLow);
    EXPECT_FALSE(debouncer.update(kLow).has_value());
    EXPECT_EQ(debouncer.update(kHigh), SwitchPosition::kHigh);
    EXPECT_FALSE(debouncer.update(kHigh).has_value());
}

TEST(SwitchDebouncerTest, JitterWithinOneSideDoesNotEmit)
{
    // The previous implementation compared raw values, so this re-sent the E-Stop set/reset.
    SwitchDebouncer debouncer(kThreshold, 0);
    debouncer.update(kHigh);

    EXPECT_FALSE(debouncer.update(kHigh - 1).has_value());
    EXPECT_FALSE(debouncer.update(kHigh + 1).has_value());
    EXPECT_FALSE(debouncer.update(kThreshold).has_value());
}

TEST(SwitchDebouncerTest, ThresholdItselfCountsAsHigh)
{
    SwitchDebouncer debouncer(kThreshold, 0);
    debouncer.update(kLow);

    EXPECT_EQ(debouncer.update(kThreshold), SwitchPosition::kHigh);
    EXPECT_EQ(debouncer.update(kThreshold - 1), SwitchPosition::kLow);
}

}  // namespace rover_crfs_teleop
