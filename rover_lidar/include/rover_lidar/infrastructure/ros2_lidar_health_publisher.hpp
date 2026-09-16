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

#ifndef ROVER_LIDAR_INFRASTRUCTURE_ROS2_LIDAR_HEALTH_PUBLISHER_HPP_
#define ROVER_LIDAR_INFRASTRUCTURE_ROS2_LIDAR_HEALTH_PUBLISHER_HPP_

#include <memory>
#include <optional>

#include "diagnostic_updater/diagnostic_updater.hpp"

#include "rover_lidar/domain/lidar_health_evaluator.hpp"
#include "rover_lidar/domain/ports/lidar_health_publisher_port.hpp"

namespace rover_lidar::infrastructure
{

unsigned char toDiagnosticLevel(domain::HealthLevel level);

/** @brief Reports the point-cloud stream health as the "Lidar status" diagnostic task. */
class Ros2LidarHealthPublisher : public domain::LidarHealthPublisherPort
{
public:
    explicit Ros2LidarHealthPublisher(
        const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater);

    void publish(const domain::LidarHealthReport & report) override;

private:
    void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status);

    std::optional<domain::LidarHealthReport> report_;
};

}  // namespace rover_lidar::infrastructure

#endif  // ROVER_LIDAR_INFRASTRUCTURE_ROS2_LIDAR_HEALTH_PUBLISHER_HPP_
