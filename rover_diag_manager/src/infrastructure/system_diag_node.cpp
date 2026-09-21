// Copyright 2025 Mechatronics Academy
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

#include "rover_diag_manager/infrastructure/system_diag_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rover_diag_manager/infrastructure/ros2_system_status_publisher.hpp"
#include "rover_diag_manager/infrastructure/system_status_msg_conversions.hpp"

namespace rover_diag_manager
{

SystemDiagNode::SystemDiagNode(
    const std::string & node_name,
    std::shared_ptr<domain::SystemMetricsSourcePort> metrics_source,
    const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
, shutdown_gate_(get_node_base_interface()->get_context(), [this]() {
    if (timer_) {
        timer_->cancel();
    }
})
{
    RCLCPP_INFO(get_logger(), "Initializing.");

    param_listener_ = std::make_shared<system_diag::ParamListener>(get_node_parameters_interface());
    const auto params = param_listener_->get_params();

    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("Rover PC");

    monitor_system_ = std::make_unique<application::MonitorSystemUseCase>(
        std::move(metrics_source),
        std::make_shared<infrastructure::Ros2SystemStatusPublisher>(*this, diagnostic_updater_));

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / params.publish_frequency)),
        std::bind(&SystemDiagNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Initialized successfully.");
}

void SystemDiagNode::timerCallback()
{
    // Re-read every tick so threshold changes via `ros2 param set` apply immediately.
    monitor_system_->tick(infrastructure::toThresholds(param_listener_->get_params()));
}

}  // namespace rover_diag_manager
