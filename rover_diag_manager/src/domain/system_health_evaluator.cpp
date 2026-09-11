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

#include "rover_diag_manager/domain/system_health_evaluator.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <utility>

namespace rover_diag_manager::domain
{

namespace
{

MetricFinding grade(std::string name, std::optional<float> value, double threshold)
{
    HealthLevel level = HealthLevel::Ok;

    if (!value) {
        level = HealthLevel::Error;
    } else if (static_cast<double>(*value) > threshold) {
        level = HealthLevel::Warn;
    }

    return MetricFinding{std::move(name), value, level};
}

std::string messageFor(HealthLevel level)
{
    switch (level) {
        case HealthLevel::Error:
            return "Detected system parameter with unknown value.";
        case HealthLevel::Warn:
            return "At least one system parameter is above the warning threshold.";
        case HealthLevel::Ok:
        default:
            return "System parameters are within acceptable limits.";
    }
}

}  // namespace

HealthReport evaluateSystemHealth(
    const SystemSample & sample, const SystemHealthThresholds & thresholds)
{
    HealthReport report;
    report.findings = {
        grade("CPU usage (%)", sample.cpu_mean_usage, thresholds.cpu_usage),
        grade("CPU temperature (°C)", sample.cpu_temperature, thresholds.cpu_temperature),
        grade("RAM memory usage (%)", sample.ram_usage, thresholds.ram_usage),
        grade("Disk memory usage (%)", sample.disk_usage, thresholds.disk_usage),
    };

    for (const auto & finding : report.findings) {
        report.level = std::max(report.level, finding.level);
    }
    report.message = messageFor(report.level);

    return report;
}

}  // namespace rover_diag_manager::domain
