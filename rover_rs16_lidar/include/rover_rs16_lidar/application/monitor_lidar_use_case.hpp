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

#ifndef ROVER_RS16_LIDAR_APPLICATION_MONITOR_LIDAR_USE_CASE_HPP_
#define ROVER_RS16_LIDAR_APPLICATION_MONITOR_LIDAR_USE_CASE_HPP_

#include <memory>
#include <mutex>

#include "rover_rs16_lidar/domain/lidar_health_evaluator.hpp"
#include "rover_rs16_lidar/domain/ports/lidar_health_publisher_port.hpp"

namespace rover_rs16_lidar::application
{

/**
 * @brief Feeds point-cloud arrivals into the evaluator and publishes the verdict on demand.
 * @details Thread-safe: onCloud() runs on the driver's frame thread while publishHealth()
 *          runs on the ROS executor thread.
 */
class MonitorLidarUseCase
{
public:
    MonitorLidarUseCase(
        domain::LidarHealthThresholds thresholds,
        std::shared_ptr<domain::LidarHealthPublisherPort> publisher);

    void onCloud(const domain::CloudSample & cloud);

    /** @brief Evaluates at `now_s` and pushes the report to the publisher port. */
    domain::LidarHealthReport publishHealth(double now_s);

private:
    mutable std::mutex mutex_;
    domain::LidarHealthEvaluator evaluator_;
    std::shared_ptr<domain::LidarHealthPublisherPort> publisher_;
};

}  // namespace rover_rs16_lidar::application

#endif  // ROVER_RS16_LIDAR_APPLICATION_MONITOR_LIDAR_USE_CASE_HPP_
