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

#include "rover_crfs_teleop/infrastructure/ros2_trigger_safety_switch.hpp"

#include <memory>

namespace rover_crfs_telop
{

Ros2TriggerSafetySwitch::Ros2TriggerSafetySwitch(rclcpp_lifecycle::LifecycleNode & node)
: logger_(node.get_logger()),
  clock_(node.get_clock()),
  e_stop_set_(node.create_client<std_srvs::srv::Trigger>("hardware_interface/sw_user_e_stop_set")),
  e_stop_reset_(
      node.create_client<std_srvs::srv::Trigger>("hardware_interface/sw_user_e_stop_reset")),
  e_stop_latch_reset_(
      node.create_client<std_srvs::srv::Trigger>("hardware_interface/sw_e_stop_latch_reset"))
{
}

void Ros2TriggerSafetySwitch::requestUserEStopSet()
{
    callTriggerService(e_stop_set_, "SW User E-Stop set");
}

void Ros2TriggerSafetySwitch::requestUserEStopReset()
{
    callTriggerService(e_stop_reset_, "SW User E-Stop reset");
}

void Ros2TriggerSafetySwitch::requestLatchReset()
{
    callTriggerService(e_stop_latch_reset_, "SW E-Stop latch reset");
}

void Ros2TriggerSafetySwitch::callTriggerService(
    const TriggerClient::SharedPtr & client, const std::string & description)
{
    if (!client->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "Service for '%s' is not available; request dropped.", description.c_str());
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // The response used to be discarded, so a refused request (e.g. the hardware interface
    // rejecting an E-Stop reset because the rover is still being commanded to move) was
    // invisible to the operator - and, because the request only re-fires on a switch change,
    // there was no retry either. Report it.
    client->async_send_request(
        request,
        [logger = logger_, description](TriggerClient::SharedFuture future) {
            const auto response = future.get();

            if (response->success) {
                RCLCPP_INFO(logger, "%s succeeded.", description.c_str());
            } else {
                RCLCPP_WARN(
                    logger, "%s was refused: %s", description.c_str(), response->message.c_str());
            }
        });
}

}  // namespace rover_crfs_telop
