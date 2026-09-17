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

#ifndef ROVER_RS16_LIDAR_INFRASTRUCTURE_ROS2_LASER_SCAN_PUBLISHER_HPP_
#define ROVER_RS16_LIDAR_INFRASTRUCTURE_ROS2_LASER_SCAN_PUBLISHER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rover_rs16_lidar/domain/laser_scan_frame.hpp"
#include "rover_rs16_lidar/domain/ports/laser_scan_publisher_port.hpp"

namespace rover_rs16_lidar::infrastructure
{

/** @brief Publishes the flattened scan as sensor_msgs/LaserScan in the sensor frame. */
class Ros2LaserScanPublisher : public domain::LaserScanPublisherPort
{
public:
    Ros2LaserScanPublisher(
        rclcpp::Node * node, const std::string & topic, const std::string & frame_id,
        std::size_t queue_size);

    void publish(const domain::LaserScanFrame & scan) override;

private:
    std::string frame_id_;
    sensor_msgs::msg::LaserScan message_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

}  // namespace rover_rs16_lidar::infrastructure

#endif  // ROVER_RS16_LIDAR_INFRASTRUCTURE_ROS2_LASER_SCAN_PUBLISHER_HPP_
