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

#ifndef ROVER_BATTERY_INFRASTRUCTURE_ROS2_BATTERY_STATE_PUBLISHER_HPP_
#define ROVER_BATTERY_INFRASTRUCTURE_ROS2_BATTERY_STATE_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_battery/domain/ports/battery_state_publisher_port.hpp"
#include "rover_battery/infrastructure/battery_msg_conversions.hpp"

namespace rover_battery::infrastructure
{

/**
 * @brief Publishes battery reports on rover_battery/battery_status and
 *        rover_battery/charging_status, and feeds the "Battery errors" / "Battery status"
 *        diagnostic tasks.
 */
class Ros2BatteryStatePublisher : public domain::BatteryStatePublisherPort
{
public:
    Ros2BatteryStatePublisher(
        rclcpp::Node & node,
        const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater);

    void publish(const domain::BatteryReport & report) override;

private:
    void logBatteryStatus(const BatteryStateMsg & battery_state);
    void logErrors(const std::string & error_msg);
    void diagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status);
    void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status);

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
    rclcpp::Publisher<ChargingStatusMsg>::SharedPtr charging_status_pub_;

    // Errors of the latest report; empty once the BMS alarms clear.
    std::string error_msg_;
    // False until the first report (real or watchdog-stale), so diagnostics read STALE, not OK.
    bool has_report_{false};
    BatteryStateMsg battery_state_;
    ChargingStatusMsg charging_status_;
};

}  // namespace rover_battery::infrastructure

#endif  // ROVER_BATTERY_INFRASTRUCTURE_ROS2_BATTERY_STATE_PUBLISHER_HPP_
