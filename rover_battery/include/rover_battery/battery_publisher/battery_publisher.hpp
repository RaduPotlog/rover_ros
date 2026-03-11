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

#ifndef ROVER_BATTERY_BATTERY_PUBLISHER_BATTERY_PUBLISHER_HPP_
#define ROVER_BATTERY_BATTERY_PUBLISHER_BATTERY_PUBLISHER_HPP_

#include <memory>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "rover_msgs/msg/charging_status.hpp"
#include "rover_battery/battery_publisher/battery_publisher.hpp"

namespace rover_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using ChargingStatusMsg = rover_msgs::msg::ChargingStatus;

class BatteryPublisher
{

public:
  
    BatteryPublisher(
        const rclcpp::Node::SharedPtr & node,
        const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
        const double battery_timeout);

    ~BatteryPublisher() {}

    void publish(
        BatteryStateMsg & battery_state_msg,
        ChargingStatusMsg & battery_charging_status,
        const std::string & error_msg);

protected:
  
    virtual void publishBatteryState(BatteryStateMsg & battery_state_msg) = 0;
    virtual void publishChargingStatus(ChargingStatusMsg & battery_charging_status) = 0;
    virtual void logErrors(const std::string & error_msg) = 0;
    virtual void diagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;
    virtual void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;

    void batteryStatusLogger(const BatteryStateMsg & battery_state);
    bool chargerConnected() const;
    std::string mapPowerSupplyStatusToString(uint8_t power_supply_status) const;

    rclcpp::Logger getLogger();
    rclcpp::Clock::SharedPtr getClock();

    rclcpp::Node::WeakPtr node_;
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;

    std::string error_msg_;
    
    BatteryStateMsg battery_status_;
    ChargingStatusMsg charging_status_;
    
private:

    bool charger_connected_;
    float battery_timeout_;
    
    rclcpp::Time last_battery_info_time_;
};

}  // namespace rover_battery

#endif  // ROVER_BATTERY_BATTERY_PUBLISHER_BATTERY_PUBLISHER_HPP_
