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

#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "rover_crfs_teleop/infrastructure/teleop_diagnostics_conversions.hpp"

namespace rover_crfs_teleop
{
namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;

TeleopDiagnostics lostLink()
{
    TeleopDiagnostics diagnostics;
    diagnostics.first_frame_received = true;
    diagnostics.link.loss_reason = LinkLossReason::kChannelsStale;
    diagnostics.health = evaluateTeleopHealth(true, diagnostics.link);
    return diagnostics;
}

bool hasKey(const diagnostic_updater::DiagnosticStatusWrapper & status, const std::string & key)
{
    for (const auto & value : status.values) {
        if (value.key == key) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST(TeleopDiagnosticsConversionsTest, ActiveLostLinkIsError)
{
    diagnostic_updater::DiagnosticStatusWrapper status;
    fillRcLinkStatus(lostLink(), true, status);

    EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
    EXPECT_TRUE(hasKey(status, "Link loss reason"));
    EXPECT_TRUE(hasKey(status, "rc/channels age (ms)"));
}

TEST(TeleopDiagnosticsConversionsTest, InactiveLostLinkIsCappedAtWarn)
{
    diagnostic_updater::DiagnosticStatusWrapper status;
    fillRcLinkStatus(lostLink(), false, status);

    EXPECT_EQ(status.level, DiagnosticStatus::WARN);
    EXPECT_NE(status.message.find("inactive"), std::string::npos);
}

TEST(TeleopDiagnosticsConversionsTest, ReachableServicesAreOk)
{
    std::vector<SafetyRequestStatus> requests(3);
    for (auto & request : requests) {
        request.description = "svc";
        request.service_ready = true;
        request.outcome = SafetyRequestOutcome::kSucceeded;
    }

    diagnostic_updater::DiagnosticStatusWrapper status;
    fillSafetyRequestsStatus(requests, status);

    EXPECT_EQ(status.level, DiagnosticStatus::OK);
}

TEST(TeleopDiagnosticsConversionsTest, RefusedRequestWarnsWithMessage)
{
    std::vector<SafetyRequestStatus> requests(1);
    requests[0].description = "SW User E-Stop reset";
    requests[0].service_ready = true;
    requests[0].outcome = SafetyRequestOutcome::kRefused;
    requests[0].message = "velocity commands are not zero";

    diagnostic_updater::DiagnosticStatusWrapper status;
    fillSafetyRequestsStatus(requests, status);

    EXPECT_EQ(status.level, DiagnosticStatus::WARN);
    bool found = false;
    for (const auto & value : status.values) {
        found = found || value.value.find("velocity commands are not zero") != std::string::npos;
    }
    EXPECT_TRUE(found);
}

TEST(TeleopDiagnosticsConversionsTest, UnreachableServiceWarns)
{
    std::vector<SafetyRequestStatus> requests(1);
    requests[0].description = "SW E-Stop latch reset";
    requests[0].service_ready = false;

    diagnostic_updater::DiagnosticStatusWrapper status;
    fillSafetyRequestsStatus(requests, status);

    EXPECT_EQ(status.level, DiagnosticStatus::WARN);
}

}  // namespace rover_crfs_teleop
