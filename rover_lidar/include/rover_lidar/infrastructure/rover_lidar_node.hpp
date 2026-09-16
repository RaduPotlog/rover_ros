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

#ifndef ROVER_LIDAR_INFRASTRUCTURE_ROVER_LIDAR_NODE_HPP_
#define ROVER_LIDAR_INFRASTRUCTURE_ROVER_LIDAR_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rover_lidar/application/monitor_lidar_use_case.hpp"
#include "rover_lidar/domain/lidar_health_evaluator.hpp"

namespace rover_lidar
{

/**
 * @brief Composition root: lidar point-cloud health diagnostics.
 * @details Plain (non-lifecycle) node on purpose — the sensor socket is owned by
 *          rover_rslidar_sdk_node; this node only observes the published cloud.
 */
class RoverLidarNode : public rclcpp::Node
{
public:
    RoverLidarNode(
        const std::string & node_name, const std::string & ns = "/",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void init();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void tickCallback();

    double nowSeconds() const;

    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    std::unique_ptr<application::MonitorLidarUseCase> monitor_lidar_;

    domain::LidarHealthThresholds health_thresholds_;
    std::string pointcloud_topic_;
    double publish_frequency_{1.0};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace rover_lidar

#endif  // ROVER_LIDAR_INFRASTRUCTURE_ROVER_LIDAR_NODE_HPP_
