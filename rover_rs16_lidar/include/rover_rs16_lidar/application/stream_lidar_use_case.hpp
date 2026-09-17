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

#ifndef ROVER_RS16_LIDAR_APPLICATION_STREAM_LIDAR_USE_CASE_HPP_
#define ROVER_RS16_LIDAR_APPLICATION_STREAM_LIDAR_USE_CASE_HPP_

#include <memory>

#include "rover_rs16_lidar/application/monitor_lidar_use_case.hpp"
#include "rover_rs16_lidar/domain/point_cloud_frame.hpp"
#include "rover_rs16_lidar/domain/ports/laser_scan_publisher_port.hpp"
#include "rover_rs16_lidar/domain/ports/point_cloud_publisher_port.hpp"
#include "rover_rs16_lidar/domain/scan_projector.hpp"

namespace rover_rs16_lidar::application
{

/**
 * @brief What happens to every revolution the sensor produces.
 * @details Publishes the cloud, projects and publishes the scan when one is wanted, and feeds
 *          the health monitor. Runs on the driver's frame thread, so all of this work stays
 *          off the ROS executor.
 */
class StreamLidarUseCase
{
public:
    /**
     * @param scan_projector `nullptr` when no LaserScan is wanted; `scan_publisher` is then
     *        unused and may be `nullptr` too.
     * @throws std::invalid_argument when a required collaborator is missing.
     */
    StreamLidarUseCase(
        std::shared_ptr<domain::PointCloudPublisherPort> cloud_publisher,
        std::shared_ptr<domain::LaserScanPublisherPort> scan_publisher,
        std::shared_ptr<domain::ScanProjector> scan_projector,
        std::shared_ptr<MonitorLidarUseCase> monitor);

    /** @param arrival_s Rover-clock arrival time; see CloudSample on why not the stamp. */
    void onFrame(const domain::PointCloudFrame & cloud, double arrival_s);

private:
    std::shared_ptr<domain::PointCloudPublisherPort> cloud_publisher_;
    std::shared_ptr<domain::LaserScanPublisherPort> scan_publisher_;
    std::shared_ptr<domain::ScanProjector> scan_projector_;
    std::shared_ptr<MonitorLidarUseCase> monitor_;
};

}  // namespace rover_rs16_lidar::application

#endif  // ROVER_RS16_LIDAR_APPLICATION_STREAM_LIDAR_USE_CASE_HPP_
