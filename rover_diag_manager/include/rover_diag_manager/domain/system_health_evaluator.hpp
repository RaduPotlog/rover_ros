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

#ifndef ROVER_DIAG_MANAGER_DOMAIN_SYSTEM_HEALTH_EVALUATOR_HPP_
#define ROVER_DIAG_MANAGER_DOMAIN_SYSTEM_HEALTH_EVALUATOR_HPP_

#include "rover_diag_manager/domain/health_report.hpp"
#include "rover_diag_manager/domain/system_sample.hpp"
#include "rover_diag_manager/domain/thresholds.hpp"

namespace rover_diag_manager::domain
{

/**
 * @brief Grades every metric of `sample` against `thresholds`.
 * @details An unknown metric is an Error, a metric strictly above its threshold is a Warn.
 *          Each metric is checked independently, so equal thresholds never shadow each other.
 */
HealthReport evaluateSystemHealth(
    const SystemSample & sample, const SystemHealthThresholds & thresholds);

}  // namespace rover_diag_manager::domain

#endif  // ROVER_DIAG_MANAGER_DOMAIN_SYSTEM_HEALTH_EVALUATOR_HPP_
