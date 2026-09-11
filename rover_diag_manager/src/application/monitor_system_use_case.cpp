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

#include "rover_diag_manager/application/monitor_system_use_case.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "rover_diag_manager/domain/system_health_evaluator.hpp"

namespace rover_diag_manager::application
{

MonitorSystemUseCase::MonitorSystemUseCase(
    std::shared_ptr<domain::SystemMetricsSourcePort> metrics_source,
    std::shared_ptr<domain::SystemStatusPublisherPort> publisher)
: metrics_source_(std::move(metrics_source))
, publisher_(std::move(publisher))
{
    if (!metrics_source_) {
        throw std::invalid_argument("MonitorSystemUseCase requires a metrics source");
    }
    if (!publisher_) {
        throw std::invalid_argument("MonitorSystemUseCase requires a publisher");
    }
}

void MonitorSystemUseCase::tick(const domain::SystemHealthThresholds & thresholds)
{
    const domain::SystemSample sample = metrics_source_->sample();

    publisher_->publish(sample, domain::evaluateSystemHealth(sample, thresholds));
}

}  // namespace rover_diag_manager::application
