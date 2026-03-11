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

#ifndef ROVER_BATTERY_BATTERY_PUBLISHER_SINGLE_BATTERY_PUBLISHER_HPP_
#define ROVER_BATTERY_BATTERY_PUBLISHER_SINGLE_BATTERY_PUBLISHER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rover_battery/battery_publisher/battery_publisher.hpp"

namespace rover_battery
{

class SingleBatteryPublisher : public BatteryPublisher
{

public:
    
    SingleBatteryPublisher(
        const rclcpp::Node::SharedPtr & node,
        const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
        const double battery_timeout);

    ~SingleBatteryPublisher() {}

protected:

    void publishBatteryState(BatteryStateMsg & battery_state_msg) override;
    void publishChargingStatus(ChargingStatusMsg & battery_state_msg) override;
    void logErrors(const std::string & error_msg) override;
    void diagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status) override;
    void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status) override;

private:
  
    rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
    rclcpp::Publisher<ChargingStatusMsg>::SharedPtr charging_status_pub_;
};

}  // namespace rover_battery

#endif  // ROVER_BATTERY_BATTERY_PUBLISHER_SINGLE_BATTERY_PUBLISHER_HPP_
