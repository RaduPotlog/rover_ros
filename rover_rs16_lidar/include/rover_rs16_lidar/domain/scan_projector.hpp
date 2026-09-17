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

#ifndef ROVER_RS16_LIDAR_DOMAIN_SCAN_PROJECTOR_HPP_
#define ROVER_RS16_LIDAR_DOMAIN_SCAN_PROJECTOR_HPP_

#include <cstddef>

#include "rover_rs16_lidar/domain/laser_scan_frame.hpp"
#include "rover_rs16_lidar/domain/lidar_settings.hpp"
#include "rover_rs16_lidar/domain/point_cloud_frame.hpp"

namespace rover_rs16_lidar::domain
{

/**
 * @brief Flattens a point cloud into the LaserScan the Nav 2 costmaps consume.
 * @details Same semantics as the pointcloud_to_laserscan node this replaces: keep the points
 *          inside a horizontal band, bin them by bearing, and report the closest return per
 *          bin. No transform is applied - the cloud is already in the sensor frame and that
 *          is the frame the scan is published in, which is exactly what the old node's
 *          target_frame did.
 */
class ScanProjector
{
public:
    /** @throws std::invalid_argument when the settings do not describe a usable scan. */
    explicit ScanProjector(ScanSettings settings);

    /** @brief Number of bins in every scan this projector produces. */
    std::size_t binCount() const { return bin_count_; }

    /** @brief Projects `cloud`; the result carries `cloud`'s timestamp. */
    LaserScanFrame project(const PointCloudFrame & cloud) const;

private:
    ScanSettings settings_;
    std::size_t bin_count_{0};
    float empty_bin_value_{0.0F};
};

}  // namespace rover_rs16_lidar::domain

#endif  // ROVER_RS16_LIDAR_DOMAIN_SCAN_PROJECTOR_HPP_
