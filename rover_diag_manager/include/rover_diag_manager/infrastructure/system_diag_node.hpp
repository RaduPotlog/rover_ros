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

#ifndef ROVER_DIAG_MANAGER_INFRASTRUCTURE_SYSTEM_DIAG_NODE_HPP_
#define ROVER_DIAG_MANAGER_INFRASTRUCTURE_SYSTEM_DIAG_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_diag_manager/application/monitor_system_use_case.hpp"
#include "rover_diag_manager/domain/ports/system_metrics_source_port.hpp"
#include "rover_diag_manager/system_diag_params.hpp"

namespace rover_diag_manager
{

/**
 * @brief Composition root: drives MonitorSystemUseCase at `publish_frequency`.
 * @details Plain (non-lifecycle) node on purpose — it owns no resource, each tick only reads
 *          ephemeral OS counters. The metrics source is injected so tests can fake it.
 */
class SystemDiagNode : public rclcpp::Node
{
public:
    /** @throws rclcpp::exceptions::InvalidParameterValueException on an invalid override. */
    SystemDiagNode(
        const std::string & node_name,
        std::shared_ptr<domain::SystemMetricsSourcePort> metrics_source,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void timerCallback();

    std::shared_ptr<system_diag::ParamListener> param_listener_;

    std::unique_ptr<application::MonitorSystemUseCase> monitor_system_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Declared last so it is destroyed first: it holds a raw pointer to the publisher's
    // diagnostic callback, and the publisher is owned by monitor_system_.
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace rover_diag_manager

#endif  // ROVER_DIAG_MANAGER_INFRASTRUCTURE_SYSTEM_DIAG_NODE_HPP_
