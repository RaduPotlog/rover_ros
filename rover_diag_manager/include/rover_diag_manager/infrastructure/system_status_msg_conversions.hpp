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

#ifndef ROVER_DIAG_MANAGER_INFRASTRUCTURE_SYSTEM_STATUS_MSG_CONVERSIONS_HPP_
#define ROVER_DIAG_MANAGER_INFRASTRUCTURE_SYSTEM_STATUS_MSG_CONVERSIONS_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "rover_msgs/msg/system_status.hpp"

#include "rover_diag_manager/domain/health_report.hpp"
#include "rover_diag_manager/domain/system_sample.hpp"
#include "rover_diag_manager/domain/thresholds.hpp"
#include "rover_diag_manager/system_diag_params.hpp"

namespace rover_diag_manager::infrastructure
{

using SystemStatusMsg = rover_msgs::msg::SystemStatus;

/** @brief Maps a sample onto the wire message; an unknown metric becomes NaN. */
SystemStatusMsg toSystemStatusMsg(
    const domain::SystemSample & sample, const builtin_interfaces::msg::Time & stamp);

/** @brief Maps a HealthLevel onto a diagnostic_msgs/DiagnosticStatus level byte. */
unsigned char toDiagnosticLevel(domain::HealthLevel level);

/** @brief Writes one KeyValue per finding (NaN when unknown) plus the summary. */
void fillDiagnosticStatus(
    const domain::HealthReport & report, diagnostic_updater::DiagnosticStatusWrapper & status);

domain::SystemHealthThresholds toThresholds(const system_diag::Params & params);

}  // namespace rover_diag_manager::infrastructure

#endif  // ROVER_DIAG_MANAGER_INFRASTRUCTURE_SYSTEM_STATUS_MSG_CONVERSIONS_HPP_
