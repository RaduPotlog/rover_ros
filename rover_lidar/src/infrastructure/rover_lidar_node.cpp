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

#include "rover_lidar/infrastructure/rover_lidar_node.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "rover_lidar/infrastructure/ros2_lidar_health_publisher.hpp"

namespace rover_lidar
{

using std::placeholders::_1;

namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describePositive(
    const std::string & description, double max_value)
{
    auto descriptor = describe(description);
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 1.0e-6;
    descriptor.floating_point_range[0].to_value = max_value;
    return descriptor;
}

}  // namespace

RoverLidarNode::RoverLidarNode(
    const std::string & node_name,
    const std::string & ns,
    const rclcpp::NodeOptions & options)
: Node(node_name, ns, options)
, diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
    const domain::LidarHealthThresholds health_defaults;

    pointcloud_topic_ = declare_parameter(
        "pointcloud_topic", std::string("rslidar_points"),
        describe("Point cloud topic published by rover_rslidar_sdk_node."));
    health_thresholds_.expected_rate_hz = declare_parameter(
        "expected_rate_hz", health_defaults.expected_rate_hz,
        describePositive("Spin rate the lidar is configured for [Hz].", 100.0));
    health_thresholds_.min_rate_ratio = declare_parameter(
        "min_rate_ratio", health_defaults.min_rate_ratio,
        describePositive("Warn below expected_rate_hz * min_rate_ratio.", 1.0));
    health_thresholds_.cloud_timeout_s = declare_parameter(
        "cloud_timeout_s", health_defaults.cloud_timeout_s,
        describePositive("Report an error when no cloud arrives for this long [s].", 600.0));
    publish_frequency_ = declare_parameter(
        "publish_frequency", 1.0,
        describePositive("Diagnostics evaluation rate [Hz].", 100.0));

    auto points_descriptor = describe("Warn below this many points per cloud.");
    points_descriptor.integer_range.resize(1);
    points_descriptor.integer_range[0].from_value = 0;
    points_descriptor.integer_range[0].to_value = 10000000;
    health_thresholds_.min_points_warn = static_cast<std::uint32_t>(
        declare_parameter<std::int64_t>(
            "min_points_warn", static_cast<std::int64_t>(health_defaults.min_points_warn),
            points_descriptor));

    // Reject inconsistent thresholds at startup, not at the first cloud.
    domain::LidarHealthEvaluator::validate(health_thresholds_);
}

void RoverLidarNode::init()
{
    diagnostic_updater_->setHardwareID("RoverLidar");

    monitor_lidar_ = std::make_unique<application::MonitorLidarUseCase>(
        health_thresholds_,
        std::make_shared<infrastructure::Ros2LidarHealthPublisher>(diagnostic_updater_));

    // Best-effort matches a reliable publisher and keeps no backlog of point clouds; only the
    // arrival time and point count are used, so dropping clouds under load is harmless here.
    cloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&RoverLidarNode::cloudCallback, this, _1));

    const auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
    tick_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RoverLidarNode::tickCallback, this));

    // Publish the STALE status immediately instead of after the first timer period.
    tickCallback();
}

void RoverLidarNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    domain::CloudSample cloud;
    // Arrival time, not msg->header.stamp: with use_lidar_clock the stamp comes from the
    // sensor's own clock and is not comparable to the rover's.
    cloud.arrival_s = nowSeconds();
    cloud.point_count = msg->width * msg->height;

    monitor_lidar_->onCloud(cloud);
}

void RoverLidarNode::tickCallback()
{
    monitor_lidar_->publishHealth(nowSeconds());
}

double RoverLidarNode::nowSeconds() const
{
    return get_clock()->now().seconds();
}

}  // namespace rover_lidar
