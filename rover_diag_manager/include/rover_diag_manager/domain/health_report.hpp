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

#ifndef ROVER_DIAG_MANAGER_DOMAIN_HEALTH_REPORT_HPP_
#define ROVER_DIAG_MANAGER_DOMAIN_HEALTH_REPORT_HPP_

#include <optional>
#include <string>
#include <vector>

namespace rover_diag_manager::domain
{

/** @brief Ordered by severity: a report's level is the worst of its findings. */
enum class HealthLevel
{
    Ok,
    Warn,
    Error,   // a metric could not be measured
};

struct MetricFinding
{
    std::string name;             // human-readable label, e.g. "CPU usage (%)"
    std::optional<float> value;
    HealthLevel level{HealthLevel::Ok};
};

struct HealthReport
{
    HealthLevel level{HealthLevel::Ok};
    std::string message;
    std::vector<MetricFinding> findings;   // fixed order: CPU usage, CPU temperature, RAM, disk
};

}  // namespace rover_diag_manager::domain

#endif  // ROVER_DIAG_MANAGER_DOMAIN_HEALTH_REPORT_HPP_
