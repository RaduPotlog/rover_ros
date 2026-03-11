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

#include "rover_battery/battery_publisher/single_battery_publisher.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "rover_battery/battery_publisher/battery_publisher.hpp"

namespace rover_battery
{

SingleBatteryPublisher::SingleBatteryPublisher(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
    const double battery_timeout)
: BatteryPublisher(std::move(node)
, std::move(diagnostic_updater)
, battery_timeout)
{
    battery_pub_ = node->create_publisher<BatteryStateMsg>("rover_battery/battery_status", 5);
    charging_status_pub_ = node->create_publisher<ChargingStatusMsg>("rover_battery/charging_status", 5);
}

void SingleBatteryPublisher::publishBatteryState(BatteryStateMsg & battery_state_msg)
{
    battery_status_ = battery_state_msg;
    
    battery_pub_->publish(battery_state_msg);
    batteryStatusLogger(battery_state_msg);
}

void SingleBatteryPublisher::publishChargingStatus(ChargingStatusMsg & charging_status_msg)
{
    charging_status_ = charging_status_msg;

    charging_status_pub_->publish(charging_status_msg);
}

void SingleBatteryPublisher::logErrors(const std::string & error_msg)
{
    if (!error_msg.empty()) {

        error_msg_ = error_msg;

        RCLCPP_ERROR_STREAM_THROTTLE(
            getLogger(), *getClock(), 10000, "Rover battery error: " << error_msg);
    }
}

void SingleBatteryPublisher::diagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status)
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

void SingleBatteryPublisher::diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    const auto charging_status = charging_status_;

    if (charging_status.charging) {
        status.add("Power supply status", "connected");
    }
    else {
        status.add("Power supply status", "disconnected");
    }

    const auto load_current = charging_status.current;
    status.add("Load current (A)", load_current);

    status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Battery status monitoring");
}

}  // namespace rover_battery
