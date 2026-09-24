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

#include "rover_safety/plugins/action/call_set_led_animation_service_node.hpp"

namespace rover_safety
{

CallSetLedAnimationService::CallSetLedAnimationService(
    const std::string& name, 
    const BT::NodeConfig& config,
    const std::string & service_name)
: nav2_behavior_tree::BtServiceNode<rover_msgs::srv::SetLedAnimation>(name, config, service_name)
{
}

BT::PortsList CallSetLedAnimationService::providedPorts() 
{
    return providedBasicPorts({
        BT::InputPort<unsigned>("id", "Animation ID to trigger."),
        BT::InputPort<std::string>("param", "Optional animation parameter."),
        BT::InputPort<bool>("repeating", "Specifies whether the animation should repeated continuously.")
    });
}

void CallSetLedAnimationService::on_tick()
{
    unsigned animation_id;

    if (!getInput<unsigned>("id", animation_id)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [id] for %s", service_name_.c_str());
        should_send_request_ = false;
        return;
    }

    request_->animation.id = static_cast<uint16_t>(animation_id);

    if (!getInput<std::string>("param", request_->animation.param)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [param] for %s", service_name_.c_str());
        should_send_request_ = false;
        return;
    }

    if (!getInput<bool>("repeating", request_->repeating)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [repeating] for %s", service_name_.c_str());
        should_send_request_ = false;
    }
}

BT::NodeStatus CallSetLedAnimationService::on_completion(
    std::shared_ptr<rover_msgs::srv::SetLedAnimation::Response> response)
{
    if (!response->success) {
        RCLCPP_ERROR(
            node_->get_logger(), "Service %s returned failure: %s",
            service_name_.c_str(), response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_safety

#include <behaviortree_cpp/bt_factory.h>
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<rover_safety::CallSetLedAnimationService>("CallSetLedAnimationService");
}
