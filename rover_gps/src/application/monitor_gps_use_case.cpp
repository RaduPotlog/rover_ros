// Copyright 2026 Mechatronics Academy
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

#include "rover_gps/application/monitor_gps_use_case.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

namespace rover_gps::application
{

MonitorGpsUseCase::MonitorGpsUseCase(
    std::shared_ptr<domain::GpsHealthPublisherPort> publisher,
    domain::GpsHealthThresholds thresholds)
: publisher_(std::move(publisher))
, evaluator_(thresholds)
{
    if (!publisher_) {
        throw std::invalid_argument("MonitorGpsUseCase requires a publisher.");
    }
}

void MonitorGpsUseCase::onFix(const domain::GnssFix & fix)
{
    evaluator_.addFix(fix);
}

void MonitorGpsUseCase::onTick(double now_s)
{
    publisher_->publish(evaluator_.evaluate(now_s));
}

}  // namespace rover_gps::application
