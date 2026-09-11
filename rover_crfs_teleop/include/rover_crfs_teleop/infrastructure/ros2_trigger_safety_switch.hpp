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

#ifndef ROVER_CRFS_TELEOP_INFRASTRUCTURE_ROS2_TRIGGER_SAFETY_SWITCH_HPP_
#define ROVER_CRFS_TELEOP_INFRASTRUCTURE_ROS2_TRIGGER_SAFETY_SWITCH_HPP_

#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rover_crfs_teleop/domain/ports.hpp"

namespace rover_crfs_teleop
{

// Drives the hardware interface's software E-Stop through its std_srvs/Trigger services.
class Ros2TriggerSafetySwitch : public SafetySwitchPort
{

public:

    explicit Ros2TriggerSafetySwitch(rclcpp_lifecycle::LifecycleNode & node);

    void requestUserEStopSet() override;

    void requestUserEStopReset() override;

    void requestLatchReset() override;

private:

    using TriggerClient = rclcpp::Client<std_srvs::srv::Trigger>;

    // Fires `client` if - and only if - it is ready. Deliberately non-blocking: this runs inside
    // the 20 ms control timer, where the previous `while (!client->wait_for_service(1s))` loop
    // stalled the whole executor (teleop included) for seconds at a time whenever a service was
    // briefly unavailable.
    void callTriggerService(const TriggerClient::SharedPtr & client, const std::string & description);

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    TriggerClient::SharedPtr e_stop_set_;
    TriggerClient::SharedPtr e_stop_reset_;
    TriggerClient::SharedPtr e_stop_latch_reset_;
};

}  // namespace rover_crfs_teleop

#endif  // ROVER_CRFS_TELEOP_INFRASTRUCTURE_ROS2_TRIGGER_SAFETY_SWITCH_HPP_
