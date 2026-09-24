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

#ifndef ROVER_SAFETY_PLUGINS_ACTION_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_
#define ROVER_SAFETY_PLUGINS_ACTION_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/behavior_tree.h>
#include <nav2_behavior_tree/bt_service_node.hpp>
#include "rclcpp/rclcpp.hpp"

#include <rover_msgs/srv/set_led_animation.hpp>

namespace rover_safety
{

/**
 * Calls rover_msgs/SetLedAnimation with the `id`, `param` and `repeating` ports. RUNNING while
 * waiting for the response, SUCCESS only when the LED server answers success=true, FAILURE on
 * success=false or server_timeout.
 */
class CallSetLedAnimationService : public nav2_behavior_tree::BtServiceNode<rover_msgs::srv::SetLedAnimation>
{

public:

    CallSetLedAnimationService(
        const std::string& name, 
        const BT::NodeConfig & config,
        const std::string & service_name = "");

    static BT::PortsList providedPorts();

    void on_tick() override;

    BT::NodeStatus on_completion(
        std::shared_ptr<rover_msgs::srv::SetLedAnimation::Response> response) override;
};

}  // namespace rover_safety

#endif  // ROVER_SAFETY_PLUGINS_ACTION_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_
