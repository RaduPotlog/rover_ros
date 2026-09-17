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

#ifndef ROVER_RS16_LIDAR_INFRASTRUCTURE_ROS2_POINT_CLOUD_PUBLISHER_HPP_
#define ROVER_RS16_LIDAR_INFRASTRUCTURE_ROS2_POINT_CLOUD_PUBLISHER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rover_rs16_lidar/domain/point_cloud_frame.hpp"
#include "rover_rs16_lidar/domain/ports/point_cloud_publisher_port.hpp"

namespace rover_rs16_lidar::infrastructure
{

/**
 * @brief Publishes a revolution as sensor_msgs/PointCloud2.
 * @details The layout is deliberately byte-for-byte what rslidar_sdk produced with
 *          POINT_TYPE_XYZI and ros_send_by_rows=false: four FLOAT32 fields
 *          (x, y, z, intensity), and width/height swapped relative to the driver's own
 *          convention so pcl::PointCloud consumers see the cloud the way they always have.
 */
class Ros2PointCloudPublisher : public domain::PointCloudPublisherPort
{
public:
    Ros2PointCloudPublisher(
        rclcpp::Node * node, const std::string & topic, const std::string & frame_id,
        std::size_t queue_size);

    void publish(const domain::PointCloudFrame & cloud) override;

private:
    std::string frame_id_;
    /// Reused between revolutions so the data buffer keeps its capacity.
    sensor_msgs::msg::PointCloud2 message_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace rover_rs16_lidar::infrastructure

#endif  // ROVER_RS16_LIDAR_INFRASTRUCTURE_ROS2_POINT_CLOUD_PUBLISHER_HPP_
