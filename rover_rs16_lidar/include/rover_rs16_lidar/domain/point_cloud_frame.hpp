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

#ifndef ROVER_RS16_LIDAR_DOMAIN_POINT_CLOUD_FRAME_HPP_
#define ROVER_RS16_LIDAR_DOMAIN_POINT_CLOUD_FRAME_HPP_

#include <cstdint>
#include <vector>

namespace rover_rs16_lidar::domain
{

/**
 * @brief One measured return, in the sensor frame.
 * @details Mirrors the RS16 point layout the rover is built for (rs_driver POINT_TYPE_XYZI):
 *          no ring index and no per-point timestamp. Intensity is widened to float here
 *          because that is what goes on the wire as a PointCloud2 FLOAT32 field; the driver
 *          itself reports it as a uint8.
 */
struct LidarPoint
{
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    float intensity{0.0F};
};

/**
 * @brief One full revolution of the lidar, as the rest of the package sees it.
 * @details The boundary type between the driver adapter and everything else: no rs_driver
 *          type and no ROS type crosses it. `width` and `height` keep the driver's own
 *          convention (`height` = laser channels, `width` = firings per revolution); the ROS
 *          adapter is what swaps them for PCL compatibility.
 */
struct PointCloudFrame
{
    std::vector<LidarPoint> points;
    std::uint32_t width{0};
    std::uint32_t height{0};
    /// Seconds since the epoch, on whichever clock `use_lidar_clock` selected.
    double timestamp_s{0.0};
    /// False when invalid returns are kept as NaN (the rover's default).
    bool is_dense{false};
    std::uint32_t seq{0};
};

}  // namespace rover_rs16_lidar::domain

#endif  // ROVER_RS16_LIDAR_DOMAIN_POINT_CLOUD_FRAME_HPP_
