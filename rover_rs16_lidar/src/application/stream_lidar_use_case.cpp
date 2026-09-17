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

#include "rover_rs16_lidar/application/stream_lidar_use_case.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>

namespace rover_rs16_lidar::application
{

StreamLidarUseCase::StreamLidarUseCase(
    std::shared_ptr<domain::PointCloudPublisherPort> cloud_publisher,
    std::shared_ptr<domain::LaserScanPublisherPort> scan_publisher,
    std::shared_ptr<domain::ScanProjector> scan_projector,
    std::shared_ptr<MonitorLidarUseCase> monitor)
: cloud_publisher_(std::move(cloud_publisher))
, scan_publisher_(std::move(scan_publisher))
, scan_projector_(std::move(scan_projector))
, monitor_(std::move(monitor))
{
    if (!cloud_publisher_) {
        throw std::invalid_argument("StreamLidarUseCase requires a point cloud publisher.");
    }
    if (!monitor_) {
        throw std::invalid_argument("StreamLidarUseCase requires a lidar monitor.");
    }
    if (scan_projector_ && !scan_publisher_) {
        throw std::invalid_argument("StreamLidarUseCase requires a scan publisher to project.");
    }
}

void StreamLidarUseCase::onFrame(const domain::PointCloudFrame & cloud, double arrival_s)
{
    cloud_publisher_->publish(cloud);

    if (scan_projector_) {
        scan_publisher_->publish(scan_projector_->project(cloud));
    }

    domain::CloudSample sample;
    sample.arrival_s = arrival_s;
    sample.point_count = static_cast<std::uint32_t>(cloud.points.size());
    monitor_->onCloud(sample);
}

}  // namespace rover_rs16_lidar::application
