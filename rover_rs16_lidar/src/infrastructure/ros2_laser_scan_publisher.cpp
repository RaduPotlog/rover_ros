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

#include "rover_rs16_lidar/infrastructure/ros2_laser_scan_publisher.hpp"

#include <cmath>
#include <cstdint>
#include <string>

namespace rover_rs16_lidar::infrastructure
{

Ros2LaserScanPublisher::Ros2LaserScanPublisher(
    rclcpp::Node * node, const std::string & topic, const std::string & frame_id,
    std::size_t queue_size)
: frame_id_(frame_id)
{
    message_.header.frame_id = frame_id_;
    publisher_ = node->create_publisher<sensor_msgs::msg::LaserScan>(
        topic, rclcpp::SensorDataQoS().keep_last(queue_size));
}

void Ros2LaserScanPublisher::publish(const domain::LaserScanFrame & scan)
{
    message_.header.frame_id = frame_id_;
    // Same split as the point cloud, so scan and cloud carry the same instant.
    const auto seconds = static_cast<std::int32_t>(std::floor(scan.timestamp_s));
    message_.header.stamp.sec = seconds;
    message_.header.stamp.nanosec =
        static_cast<std::uint32_t>(std::lround((scan.timestamp_s - seconds) * 1e9));

    message_.angle_min = static_cast<float>(scan.angle_min);
    message_.angle_max = static_cast<float>(scan.angle_max);
    message_.angle_increment = static_cast<float>(scan.angle_increment);
    message_.time_increment = static_cast<float>(scan.time_increment);
    message_.scan_time = static_cast<float>(scan.scan_time);
    message_.range_min = static_cast<float>(scan.range_min);
    message_.range_max = static_cast<float>(scan.range_max);
    message_.ranges = scan.ranges;
    // The RS16 reports intensity per point, but a flattened beam has no single value for it.
    message_.intensities.clear();

    publisher_->publish(message_);
}

}  // namespace rover_rs16_lidar::infrastructure
