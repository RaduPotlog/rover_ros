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

#ifndef ROVER_RS16_LIDAR_INFRASTRUCTURE_ROVER_RS16_LIDAR_NODE_HPP_
#define ROVER_RS16_LIDAR_INFRASTRUCTURE_ROVER_RS16_LIDAR_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_rs16_lidar/application/monitor_lidar_use_case.hpp"
#include "rover_rs16_lidar/application/stream_lidar_use_case.hpp"
#include "rover_rs16_lidar/domain/lidar_health_evaluator.hpp"
#include "rover_rs16_lidar/domain/lidar_settings.hpp"
#include "rover_rs16_lidar/domain/ports/lidar_source_port.hpp"

namespace rover_rs16_lidar
{

/**
 * @brief Composition root: drives the RS16 and publishes its cloud, scan and health.
 * @details Plain (non-lifecycle) node, matching every other rover_* sensor package. The node
 *          does own the UDP sockets now that the driver runs in-process, so the sensor is
 *          opened in init() and closed in the destructor rather than in the constructor.
 */
class RoverRs16LidarNode : public rclcpp::Node
{
public:
    /** @throws std::invalid_argument when the parameter overrides are inconsistent. */
    RoverRs16LidarNode(
        const std::string & node_name, const std::string & ns = "/",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~RoverRs16LidarNode() override;

    /**
     * @brief Builds the pipeline and starts streaming.
     * @param source Sensor to read from; `nullptr` uses the real RS16 over rs_driver. Tests
     *        inject a stub here so the whole pipeline runs with no hardware on the network.
     * @throws std::runtime_error when the source cannot be started.
     */
    void init(std::shared_ptr<domain::LidarSourcePort> source = nullptr);

    const domain::LidarSettings & settings() const { return settings_; }

private:
    void declareParameters();

    void onFrame(const domain::PointCloudFrame & cloud);

    void onSourceError(const std::string & message);

    void tickCallback();

    double nowSeconds() const;

    domain::LidarSettings settings_;
    domain::LidarHealthThresholds health_thresholds_;
    double publish_frequency_{1.0};

    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    std::shared_ptr<application::MonitorLidarUseCase> monitor_lidar_;
    std::unique_ptr<application::StreamLidarUseCase> stream_lidar_;
    std::shared_ptr<domain::LidarSourcePort> source_;

    rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace rover_rs16_lidar

#endif  // ROVER_RS16_LIDAR_INFRASTRUCTURE_ROVER_RS16_LIDAR_NODE_HPP_
