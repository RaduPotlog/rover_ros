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


#include <string>

#include "gtest/gtest.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "rover_led/infrastructure/led_controller_diagnostics.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using rover_led::fillLedControllerStatus;
using rover_led::LayerStatus;
using rover_led::LedControllerDiagnostics;
using rover_led::LedStateSnapshot;

namespace
{

LedStateSnapshot oneSegment()
{
    LayerStatus ready;
    ready.info = {1, "READY", ""};
    ready.repeating = true;
    ready.progress = 0.5f;

    LedStateSnapshot snapshot;
    snapshot.segments.push_back(
        {"front", 1, {{rover_led::ERROR, std::nullopt}, {rover_led::STATE, ready}}});
    return snapshot;
}

std::string valueOf(const diagnostic_updater::DiagnosticStatusWrapper & status, const std::string & key)
{
    for (const auto & value : status.values) {
        if (value.key == key) {
            return value.value;
        }
    }
    return "<missing>";
}

}  // namespace

TEST(LedControllerDiagnostics, HealthyControllerIsOkAndNamesTheVisibleAnimation)
{
    diagnostic_updater::DiagnosticStatusWrapper status;
    fillLedControllerStatus(oneSegment(), LedControllerDiagnostics{}, status);

    EXPECT_EQ(status.level, DiagnosticStatus::OK);
    EXPECT_EQ(valueOf(status, "Segment front (channel 1)"), "READY [STATE, 50%, repeating]");
}

TEST(LedControllerDiagnostics, IdleSegmentIsReportedIdle)
{
    LedStateSnapshot snapshot;
    snapshot.segments.push_back({"rear", 2, {{rover_led::STATE, std::nullopt}}});

    diagnostic_updater::DiagnosticStatusWrapper status;
    fillLedControllerStatus(snapshot, LedControllerDiagnostics{}, status);

    EXPECT_EQ(valueOf(status, "Segment rear (channel 2)"), "idle");
}

TEST(LedControllerDiagnostics, RenderErrorIsError)
{
    LedControllerDiagnostics diagnostics;
    diagnostics.render_error = "panel size mismatch";
    diagnostics.segment_errors = 1;

    diagnostic_updater::DiagnosticStatusWrapper status;
    fillLedControllerStatus(oneSegment(), diagnostics, status);

    EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
}

TEST(LedControllerDiagnostics, SegmentErrorsAndUnavailableTypesWarn)
{
    LedControllerDiagnostics segment_failure;
    segment_failure.segment_errors = 2;
    diagnostic_updater::DiagnosticStatusWrapper failed;
    fillLedControllerStatus(oneSegment(), segment_failure, failed);
    EXPECT_EQ(failed.level, DiagnosticStatus::WARN);

    LedControllerDiagnostics bad_config;
    bad_config.unavailable_animations = 1;
    diagnostic_updater::DiagnosticStatusWrapper config;
    fillLedControllerStatus(oneSegment(), bad_config, config);
    EXPECT_EQ(config.level, DiagnosticStatus::WARN);
}

TEST(LedControllerDiagnostics, RejectedRequestIsInformationalOnly)
{
    LedControllerDiagnostics diagnostics;
    diagnostics.last_rejected_request = "id 99: unknown animation";

    diagnostic_updater::DiagnosticStatusWrapper status;
    fillLedControllerStatus(oneSegment(), diagnostics, status);

    EXPECT_EQ(status.level, DiagnosticStatus::OK);
    EXPECT_EQ(valueOf(status, "Last rejected animation request"), "id 99: unknown animation");
}
