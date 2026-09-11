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

#include "rover_diag_manager/infrastructure/ros2_system_status_publisher.hpp"

#include <memory>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace rover_diag_manager::infrastructure
{

Ros2SystemStatusPublisher::Ros2SystemStatusPublisher(
    rclcpp::Node & node,
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater)
: clock_(node.get_clock())
{
    system_status_pub_ = node.create_publisher<SystemStatusMsg>("system_status", 10);

    diagnostic_updater->add("OS status", this, &Ros2SystemStatusPublisher::diagnoseSystem);
}

void Ros2SystemStatusPublisher::publish(
    const domain::SystemSample & sample, const domain::HealthReport & health)
{
    system_status_pub_->publish(toSystemStatusMsg(sample, clock_->now()));

    last_report_ = health;
}

void Ros2SystemStatusPublisher::diagnoseSystem(
    diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (!last_report_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No system sample yet.");
        return;
    }

    fillDiagnosticStatus(*last_report_, status);
}

}  // namespace rover_diag_manager::infrastructure
