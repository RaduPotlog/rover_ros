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

#include "rover_crfs_teleop/domain/teleop_health.hpp"

namespace rover_crfs_teleop
{

TEST(TeleopHealthTest, WaitingForFirstFrameIsWarn)
{
    const auto report = evaluateTeleopHealth(false, LinkHealthSnapshot{});

    EXPECT_EQ(report.level, HealthLevel::kWarn);
}

TEST(TeleopHealthTest, LostLinkIsWarnWithReason)
{
    LinkHealthSnapshot link;
    link.loss_reason = LinkLossReason::kLowLinkQuality;

    const auto report = evaluateTeleopHealth(true, link);

    EXPECT_EQ(report.level, HealthLevel::kWarn);
    EXPECT_NE(report.message.find(toString(LinkLossReason::kLowLinkQuality)), std::string::npos);
}

TEST(TeleopHealthTest, HealthyLinkIsOk)
{
    LinkHealthSnapshot link;
    link.loss_reason = LinkLossReason::kNone;

    EXPECT_EQ(evaluateTeleopHealth(true, link).level, HealthLevel::kOk);
}

}  // namespace rover_crfs_teleop
