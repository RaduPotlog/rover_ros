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


#include "rover_safety/infrastructure/safety_diagnostics.hpp"

#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace rover_safety::infrastructure
{

namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;

const char * toString(const BT::NodeStatus tree_status)
{
    switch (tree_status) {
        case BT::NodeStatus::IDLE: return "IDLE";
        case BT::NodeStatus::RUNNING: return "RUNNING";
        case BT::NodeStatus::SUCCESS: return "SUCCESS";
        case BT::NodeStatus::FAILURE: return "FAILURE";
        case BT::NodeStatus::SKIPPED: return "SKIPPED";
    }
    return "UNKNOWN";
}

}  // namespace

unsigned char toDiagnosticLevel(const domain::HealthLevel level)
{
    switch (level) {
        case domain::HealthLevel::Error: return DiagnosticStatus::ERROR;
        case domain::HealthLevel::Warn: return DiagnosticStatus::WARN;
        case domain::HealthLevel::Ok:
        default: return DiagnosticStatus::OK;
    }
}

std::optional<double> ageSeconds(const std::optional<SteadyTime> & stamp, const SteadyTime now)
{
    if (!stamp.has_value()) {
        return std::nullopt;
    }
    return std::chrono::duration<double>(now - *stamp).count();
}

void fillSafetyInputsStatus(
    const bool subscribed, const std::vector<domain::SafetyInput> & inputs,
    diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (!subscribed) {
        status.summary(DiagnosticStatus::WARN, "Inputs not subscribed: node is not configured.");
        return;
    }

    for (const auto & input : inputs) {
        if (input.age_s.has_value()) {
            status.add(input.name + " age (s)", *input.age_s);
        } else {
            status.add(input.name + " age (s)", "never received");
        }
    }

    const auto health = domain::evaluateSafetyInputs(inputs);
    status.summary(toDiagnosticLevel(health.level), health.message);
}

void fillBehaviorTreeStatus(
    const bool configured, const bool ticking, const BT::NodeStatus tree_status,
    const unsigned failed_configure_attempts, const std::string & last_configure_error,
    diagnostic_updater::DiagnosticStatusWrapper & status)
{
    status.add("Ticking", ticking);
    status.add("Last tree status", toString(tree_status));
    status.add("Failed configure attempts", failed_configure_attempts);

    if (!configured && !last_configure_error.empty()) {
        status.add("Last configure error", last_configure_error);
        status.summary(
            DiagnosticStatus::ERROR,
            "Behavior tree not configured: " + last_configure_error);
    } else if (!configured) {
        status.summary(DiagnosticStatus::WARN, "Behavior tree not configured.");
    } else if (!ticking) {
        status.summary(
            DiagnosticStatus::WARN, "Behavior tree not ticking: waiting for required inputs.");
    } else if (tree_status == BT::NodeStatus::FAILURE) {
        status.summary(DiagnosticStatus::WARN, "Behavior tree returned FAILURE.");
    } else {
        status.summary(DiagnosticStatus::OK, "Behavior tree ticking.");
    }
}

void fillShutdownStatus(
    const domain::ShutdownSequence & sequence, diagnostic_updater::DiagnosticStatusWrapper & status)
{
    status.add("State", domain::toString(sequence.state()));
    status.add("Attempts", sequence.attempts());

    const auto level = toDiagnosticLevel(sequence.healthLevel());

    switch (sequence.state()) {
        case domain::ShutdownState::Idle:
            status.summary(level, "No shutdown requested.");
            break;
        case domain::ShutdownState::InProgress:
            status.add("Reason", sequence.reason());
            status.summary(level, "Shutting down the ROS controller: " + sequence.reason());
            break;
        case domain::ShutdownState::Succeeded:
            status.add("Reason", sequence.reason());
            status.summary(level, "Power-off requested: " + sequence.reason());
            break;
        case domain::ShutdownState::Failed:
            status.add("Reason", sequence.reason());
            status.add("Error", sequence.detail());
            status.summary(level, "Shutdown failed: " + sequence.detail());
            break;
    }
}

}  // namespace rover_safety::infrastructure
