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

#ifndef ROVER_GPS_INFRASTRUCTURE_ROS2_GPS_HEALTH_PUBLISHER_HPP_
#define ROVER_GPS_INFRASTRUCTURE_ROS2_GPS_HEALTH_PUBLISHER_HPP_

#include <memory>
#include <optional>

#include "diagnostic_updater/diagnostic_updater.hpp"

#include "rover_gps/domain/gps_health_evaluator.hpp"
#include "rover_gps/domain/ports/gps_health_publisher_port.hpp"

namespace rover_gps::infrastructure
{

/** @brief Reports the GNSS stream health as the "GPS fix" diagnostic task. */
class Ros2GpsHealthPublisher : public domain::GpsHealthPublisherPort
{
public:
    explicit Ros2GpsHealthPublisher(
        const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater);

    void publish(const domain::GpsHealthReport & report) override;

private:
    void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status);

    std::optional<domain::GpsHealthReport> report_;
};

}  // namespace rover_gps::infrastructure

#endif  // ROVER_GPS_INFRASTRUCTURE_ROS2_GPS_HEALTH_PUBLISHER_HPP_
