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

#include "rover_battery/infrastructure/ros2_battery_state_publisher.hpp"

#include <cmath>
#include <memory>
#include <string>

namespace rover_battery::infrastructure
{

namespace
{
constexpr int kLogThrottleMs = 10000;

std::string percentageText(const BatteryStateMsg & battery_state)
{
    return std::to_string(static_cast<int>(std::round(battery_state.percentage * 100.0))) + "%.";
}
}  // namespace

Ros2BatteryStatePublisher::Ros2BatteryStatePublisher(
    rclcpp::Node & node,
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater)
: logger_(node.get_logger())
, clock_(node.get_clock())
{
    battery_pub_ = node.create_publisher<BatteryStateMsg>("rover_battery/battery_status", 5);
    charging_status_pub_ =
        node.create_publisher<ChargingStatusMsg>("rover_battery/charging_status", 5);

    diagnostic_updater->add("Battery errors", this, &Ros2BatteryStatePublisher::diagnoseErrors);
    diagnostic_updater->add("Battery status", this, &Ros2BatteryStatePublisher::diagnoseStatus);
}

void Ros2BatteryStatePublisher::publish(const domain::BatteryReport & report)
{
    const BatteryStateMsg battery_state = toBatteryStateMsg(report.reading);
    battery_pub_->publish(battery_state);
    logBatteryStatus(battery_state);

    charging_status_ = toChargingStatusMsg(report.charging);
    charging_status_pub_->publish(charging_status_);

    logErrors(joinErrors(report.errors));
}

void Ros2BatteryStatePublisher::logBatteryStatus(const BatteryStateMsg & battery_state)
{
    switch (battery_state.power_supply_status) {
        case BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING:
            RCLCPP_INFO_STREAM_THROTTLE(logger_, *clock_, kLogThrottleMs,
                "The robot is not charging. Current battery percentage: " <<
                percentageText(battery_state));
            break;

        case BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING:
            RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, kLogThrottleMs,
                "The robot is charging. Current battery percentage: " <<
                percentageText(battery_state));
            break;

        case BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING:
            RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, kLogThrottleMs,
                "The robot is discharging. Current battery percentage: " <<
                percentageText(battery_state));
            break;

        case BatteryStateMsg::POWER_SUPPLY_STATUS_FULL:
            RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, kLogThrottleMs,
                "The battery is fully charged. Robot can be disconnected from the charger.");
            break;

        default:
            break;
    }
}

void Ros2BatteryStatePublisher::logErrors(const std::string & error_msg)
{
    if (error_msg.empty()) {
        return;
    }

    error_msg_ = error_msg;

    RCLCPP_ERROR_STREAM_THROTTLE(
        logger_, *clock_, kLogThrottleMs, "Rover battery error: " << error_msg);
}

void Ros2BatteryStatePublisher::diagnoseErrors(
    diagnostic_updater::DiagnosticStatusWrapper & status)
{
    unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::OK};
    std::string message{"Battery has no errors"};

    if (!error_msg_.empty()) {
        error_level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
        message = "Battery has error";

        status.add("Error message", error_msg_);
    }

    status.summary(error_level, message);
}

void Ros2BatteryStatePublisher::diagnoseStatus(
    diagnostic_updater::DiagnosticStatusWrapper & status)
{
    status.add("Power supply status", charging_status_.charging ? "connected" : "disconnected");
    status.add("Load current (A)", charging_status_.current);

    status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Battery status monitoring");
}

}  // namespace rover_battery::infrastructure
