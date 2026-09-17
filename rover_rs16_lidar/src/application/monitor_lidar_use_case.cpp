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

#include "rover_rs16_lidar/application/monitor_lidar_use_case.hpp"

#include <mutex>
#include <stdexcept>
#include <utility>

namespace rover_rs16_lidar::application
{

MonitorLidarUseCase::MonitorLidarUseCase(
    domain::LidarHealthThresholds thresholds,
    std::shared_ptr<domain::LidarHealthPublisherPort> publisher)
: evaluator_(thresholds), publisher_(std::move(publisher))
{
    if (!publisher_) {
        throw std::invalid_argument("MonitorLidarUseCase requires a health publisher.");
    }
}

void MonitorLidarUseCase::onCloud(const domain::CloudSample & cloud)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    evaluator_.addCloud(cloud);
}

domain::LidarHealthReport MonitorLidarUseCase::publishHealth(double now_s)
{
    const auto report = [&] {
        const std::lock_guard<std::mutex> lock(mutex_);
        return evaluator_.evaluate(now_s);
    }();
    publisher_->publish(report);
    return report;
}

}  // namespace rover_rs16_lidar::application
