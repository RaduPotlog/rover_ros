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

#include "rover_diag_manager/infrastructure/system_status_msg_conversions.hpp"

#include <limits>
#include <optional>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace rover_diag_manager::infrastructure
{

namespace
{

float orNaN(const std::optional<float> & value)
{
    return value.value_or(std::numeric_limits<float>::quiet_NaN());
}

}  // namespace

SystemStatusMsg toSystemStatusMsg(
    const domain::SystemSample & sample, const builtin_interfaces::msg::Time & stamp)
{
    SystemStatusMsg message;

    message.header.stamp = stamp;
    message.cpu_percent = sample.core_usages;
    message.avg_load_percent = orNaN(sample.cpu_mean_usage);
    message.cpu_temp = orNaN(sample.cpu_temperature);
    message.ram_usage_percent = orNaN(sample.ram_usage);
    // TODO(rover_msgs): "disc" typo is part of the published interface; rename needs a msg bump.
    message.disc_usage_percent = orNaN(sample.disk_usage);

    return message;
}

unsigned char toDiagnosticLevel(domain::HealthLevel level)
{
    using diagnostic_msgs::msg::DiagnosticStatus;

    switch (level) {
        case domain::HealthLevel::Error:
            return DiagnosticStatus::ERROR;
        case domain::HealthLevel::Warn:
            return DiagnosticStatus::WARN;
        case domain::HealthLevel::Ok:
        default:
            return DiagnosticStatus::OK;
    }
}

void fillDiagnosticStatus(
    const domain::HealthReport & report, diagnostic_updater::DiagnosticStatusWrapper & status)
{
    for (const auto & finding : report.findings) {
        status.add(finding.name, orNaN(finding.value));
    }

    status.summary(toDiagnosticLevel(report.level), report.message);
}

domain::SystemHealthThresholds toThresholds(const system_diag::Params & params)
{
    domain::SystemHealthThresholds thresholds;
    thresholds.cpu_usage = params.cpu_usage_warn_threshold;
    thresholds.cpu_temperature = params.cpu_temperature_warn_threshold;
    thresholds.ram_usage = params.ram_usage_warn_threshold;
    thresholds.disk_usage = params.disk_usage_warn_threshold;
    return thresholds;
}

}  // namespace rover_diag_manager::infrastructure
