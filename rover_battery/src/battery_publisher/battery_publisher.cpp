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

#include "rover_battery/battery_publisher/battery_publisher.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rover_battery
{

BatteryPublisher::BatteryPublisher(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
  const double battery_timeout)
: node_(std::move(node))
, diagnostic_updater_(std::move(diagnostic_updater))
, battery_timeout_(battery_timeout)
{
    charger_connected_ = false;
    last_battery_info_time_ = rclcpp::Time(std::int64_t(0), RCL_ROS_TIME);

    diagnostic_updater_->add("Battery errors", this, &BatteryPublisher::diagnoseErrors);
    diagnostic_updater_->add("Battery status", this, &BatteryPublisher::diagnoseStatus);
}

void BatteryPublisher::publish(
    BatteryStateMsg & battery_state_msg, 
    ChargingStatusMsg & battery_charging_status,
    const std::string & error_msg)
{
    try {
        last_battery_info_time_ = getClock()->now();
    } catch (const std::runtime_error & e) {
        RCLCPP_WARN_STREAM_THROTTLE(
            getLogger(), *getClock(), 1000,
            "An exception occurred while reading battery data: " << e.what());

    diagnostic_updater_->broadcast(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Error reading battery data: " + std::string(e.what()));
    }

    this->publishBatteryState(battery_state_msg);
    this->publishChargingStatus(battery_charging_status);
    this->logErrors(error_msg);
}

void BatteryPublisher::batteryStatusLogger(const BatteryStateMsg & battery_state)
{
    std::string msg{};

    switch (battery_state.power_supply_status) {
        case BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING:
            msg = "The robot is not charging. Current battery percentage: " +
                std::to_string(static_cast<int>(round(battery_state.percentage * 100.0))) + "%.";
            RCLCPP_INFO_STREAM_THROTTLE(getLogger(), *getClock(), 600000, msg);
            break;

        case BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING:
            msg = "The robot is charging. Current battery percentage: " +
                std::to_string(static_cast<int>(round(battery_state.percentage * 100.0))) + "%.";
            RCLCPP_WARN_STREAM_THROTTLE(getLogger(), *getClock(), 10000, msg);
            break;

        case BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING:
            msg = "The robot is discharging. Current battery percentage: " +
                std::to_string(static_cast<int>(round(battery_state.percentage * 100.0))) + "%.";
            RCLCPP_WARN_STREAM_THROTTLE(getLogger(), *getClock(), 10000, msg);
            break;

        case BatteryStateMsg::POWER_SUPPLY_STATUS_FULL:
            msg = "The battery is fully charged. Robot can be disconnected from the charger.";
            RCLCPP_WARN_STREAM_THROTTLE(getLogger(), *getClock(), 600000, msg);
            break;

        default:
            break;
    }
}

bool BatteryPublisher::chargerConnected() const 
{ 
    return charger_connected_; 
}

std::string BatteryPublisher::mapPowerSupplyStatusToString(uint8_t power_supply_status) const
{
    switch (power_supply_status) {
        case BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN:
            return "Unknown";
        case BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING:
            return "Charging";
        case BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING:
            return "Discharging";
        case BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING:
            return "Plugged and not charging";
        case BatteryStateMsg::POWER_SUPPLY_STATUS_FULL:
            return "Battery full";
        default:
            return "Invalid status";
    }
}

rclcpp::Logger BatteryPublisher::getLogger()
{
    if (auto node = node_.lock()) {
        return node->get_logger();
    }

    return rclcpp::get_logger("battery_publisher");
}

rclcpp::Clock::SharedPtr BatteryPublisher::getClock()
{
    if (auto node = node_.lock()) {
        return node->get_clock();
    }
    
    return std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

}  // namespace rover_battery
