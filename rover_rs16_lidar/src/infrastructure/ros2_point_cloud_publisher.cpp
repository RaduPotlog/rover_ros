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

#include "rover_rs16_lidar/infrastructure/ros2_point_cloud_publisher.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>

namespace rover_rs16_lidar::infrastructure
{

namespace
{

constexpr std::uint32_t kFieldSize = 4;  // every field is a FLOAT32
constexpr std::uint32_t kPointStep = 4 * kFieldSize;

// The memcpy below is only correct while LidarPoint is exactly the wire layout.
static_assert(sizeof(domain::LidarPoint) == kPointStep, "LidarPoint must be 16 packed bytes");
static_assert(offsetof(domain::LidarPoint, x) == 0, "LidarPoint layout changed");
static_assert(offsetof(domain::LidarPoint, y) == 4, "LidarPoint layout changed");
static_assert(offsetof(domain::LidarPoint, z) == 8, "LidarPoint layout changed");
static_assert(offsetof(domain::LidarPoint, intensity) == 12, "LidarPoint layout changed");
static_assert(std::is_trivially_copyable<domain::LidarPoint>::value, "LidarPoint must be POD");

sensor_msgs::msg::PointField makeField(const std::string & name, std::uint32_t offset)
{
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    return field;
}

}  // namespace

Ros2PointCloudPublisher::Ros2PointCloudPublisher(
    rclcpp::Node * node, const std::string & topic, const std::string & frame_id,
    std::size_t queue_size)
: frame_id_(frame_id)
{
    message_.fields = {
        makeField("x", 0 * kFieldSize),
        makeField("y", 1 * kFieldSize),
        makeField("z", 2 * kFieldSize),
        makeField("intensity", 3 * kFieldSize),
    };
    message_.point_step = kPointStep;
    message_.is_bigendian = false;
    message_.header.frame_id = frame_id_;

    // Sensor-data QoS: a stale revolution is worth less than a fresh one, and every consumer
    // in the rover (Nav 2 costmaps, pointcloud_crop_box) subscribes best-effort.
    publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS().keep_last(queue_size));
}

void Ros2PointCloudPublisher::publish(const domain::PointCloudFrame & cloud)
{
    // Swapped on purpose: the driver counts height as laser channels, but every existing
    // consumer has seen these the other way round since rslidar_sdk did the same swap.
    message_.width = cloud.height;
    message_.height = cloud.width;
    message_.row_step = message_.width * message_.point_step;
    message_.is_dense = cloud.is_dense;

    // Split rather than rclcpp::Time(seconds) so the rounding matches rslidar_sdk exactly.
    const auto seconds = static_cast<std::int32_t>(std::floor(cloud.timestamp_s));
    message_.header.stamp.sec = seconds;
    message_.header.stamp.nanosec =
        static_cast<std::uint32_t>(std::lround((cloud.timestamp_s - seconds) * 1e9));
    message_.header.frame_id = frame_id_;

    const std::size_t byte_count = cloud.points.size() * kPointStep;
    message_.data.resize(byte_count);
    if (byte_count > 0) {
        std::memcpy(message_.data.data(), cloud.points.data(), byte_count);
    }

    publisher_->publish(message_);
}

}  // namespace rover_rs16_lidar::infrastructure
