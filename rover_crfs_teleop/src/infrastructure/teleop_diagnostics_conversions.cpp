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


#include "rover_crfs_teleop/infrastructure/teleop_diagnostics_conversions.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace rover_crfs_teleop
{

namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;

const char * toString(const std::optional<SwitchPosition> & position)
{
    if (!position.has_value()) {
        return "unknown";
    }
    return *position == SwitchPosition::kLow ? "low" : "high";
}

const char * toString(const SafetyRequestOutcome outcome)
{
    switch (outcome) {
        case SafetyRequestOutcome::kNone: return "never requested";
        case SafetyRequestOutcome::kUnavailable: return "dropped (service unavailable)";
        case SafetyRequestOutcome::kPending: return "pending";
        case SafetyRequestOutcome::kSucceeded: return "succeeded";
        case SafetyRequestOutcome::kRefused: return "refused";
    }
    return "unknown";
}

}  // namespace

unsigned char toDiagnosticLevel(const HealthLevel level)
{
    switch (level) {
        case HealthLevel::kWarn: return DiagnosticStatus::WARN;
        case HealthLevel::kOk:
        default: return DiagnosticStatus::OK;
    }
}

void fillRcLinkStatus(
    const TeleopDiagnostics & diagnostics, const bool active,
    diagnostic_updater::DiagnosticStatusWrapper & status)
{
    const auto & link = diagnostics.link;

    status.add("Link loss reason", toString(link.loss_reason));
    status.add(
        "rc/channels age (ms)",
        link.channels_age ? std::to_string(link.channels_age->count()) : std::string("never"));
    status.add("Link stats required", link.require_link_stats);
    status.add(
        "rc/link age (ms)",
        link.link_stats_age ? std::to_string(link.link_stats_age->count()) : std::string("never"));
    status.add(
        "Uplink link quality (%)",
        link.link_quality ? std::to_string(*link.link_quality) : std::string("never"));
    status.add("Link quality OK", link.link_quality_ok);
    status.add("Commanded linear x (m/s)", diagnostics.last_command.linear_x);
    status.add("Commanded angular z (rad/s)", diagnostics.last_command.angular_z);
    status.add("E-Stop switch", toString(diagnostics.e_stop_switch));
    status.add("E-Stop latch reset switch", toString(diagnostics.latch_reset_switch));
    status.add("Teleop active", active);

    if (active) {
        status.summary(toDiagnosticLevel(diagnostics.health.level), diagnostics.health.message);
        return;
    }

    const unsigned char level = diagnostics.health.level == HealthLevel::kOk ?
        DiagnosticStatus::OK : DiagnosticStatus::WARN;
    status.summary(level, "Teleop inactive. " + diagnostics.health.message);
}

void fillSafetyRequestsStatus(
    const std::vector<SafetyRequestStatus> & requests,
    diagnostic_updater::DiagnosticStatusWrapper & status)
{
    unsigned char level = DiagnosticStatus::OK;
    std::string message = "E-Stop services reachable.";

    for (const auto & request : requests) {
        status.add(request.description + ": service ready", request.service_ready);

        std::string outcome = toString(request.outcome);
        if (request.outcome == SafetyRequestOutcome::kRefused && !request.message.empty()) {
            outcome += ": " + request.message;
        }
        status.add(request.description + ": last request", outcome);

        if (request.outcome == SafetyRequestOutcome::kRefused ||
            request.outcome == SafetyRequestOutcome::kUnavailable)
        {
            level = DiagnosticStatus::WARN;
            message = "Last " + request.description + " request " + toString(request.outcome) + ".";
        } else if (!request.service_ready && level == DiagnosticStatus::OK) {
            level = DiagnosticStatus::WARN;
            message = "E-Stop service unavailable: " + request.description + ".";
        }
    }

    status.summary(level, message);
}

}  // namespace rover_crfs_teleop
