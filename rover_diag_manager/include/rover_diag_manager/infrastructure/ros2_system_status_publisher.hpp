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

#ifndef ROVER_DIAG_MANAGER_INFRASTRUCTURE_ROS2_SYSTEM_STATUS_PUBLISHER_HPP_
#define ROVER_DIAG_MANAGER_INFRASTRUCTURE_ROS2_SYSTEM_STATUS_PUBLISHER_HPP_

#include <memory>
#include <optional>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_diag_manager/domain/ports/system_status_publisher_port.hpp"
#include "rover_diag_manager/infrastructure/system_status_msg_conversions.hpp"

namespace rover_diag_manager::infrastructure
{

/**
 * @brief Publishes samples on system_status and feeds the "OS status" diagnostic task.
 * @details The diagnostic task only formats the last published report — it never samples.
 *          publish() and the diagnostic task both run on the node's executor thread; the node
 *          uses the default (mutually exclusive) callback group, so no lock is needed.
 */
class Ros2SystemStatusPublisher : public domain::SystemStatusPublisherPort
{
public:
    Ros2SystemStatusPublisher(
        rclcpp::Node & node,
        const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater);

    void publish(const domain::SystemSample & sample, const domain::HealthReport & health) override;

private:
    void diagnoseSystem(diagnostic_updater::DiagnosticStatusWrapper & status);

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<SystemStatusMsg>::SharedPtr system_status_pub_;

    std::optional<domain::HealthReport> last_report_;
};

}  // namespace rover_diag_manager::infrastructure

#endif  // ROVER_DIAG_MANAGER_INFRASTRUCTURE_ROS2_SYSTEM_STATUS_PUBLISHER_HPP_
