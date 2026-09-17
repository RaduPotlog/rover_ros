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

#ifndef ROVER_RS16_LIDAR_DOMAIN_LASER_SCAN_FRAME_HPP_
#define ROVER_RS16_LIDAR_DOMAIN_LASER_SCAN_FRAME_HPP_

#include <vector>

namespace rover_rs16_lidar::domain
{

/**
 * @brief A 2D scan, field for field what sensor_msgs/LaserScan carries.
 * @details Kept ROS-free so the projection can be unit tested on its own. Angles are in
 *          radians, ranges in metres, times in seconds.
 */
struct LaserScanFrame
{
    double timestamp_s{0.0};
    double angle_min{0.0};
    double angle_max{0.0};
    double angle_increment{0.0};
    double time_increment{0.0};
    double scan_time{0.0};
    double range_min{0.0};
    double range_max{0.0};
    std::vector<float> ranges;
};

}  // namespace rover_rs16_lidar::domain

#endif  // ROVER_RS16_LIDAR_DOMAIN_LASER_SCAN_FRAME_HPP_
