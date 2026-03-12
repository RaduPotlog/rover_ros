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
    if (!getInput<std::string>("service_name", service_name_)) {
        throw BT::RuntimeError("Missing required input [service_name]");
    }

    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList CallSetLedAnimationService::providedPorts() 
{
    return {
        BT::InputPort<std::string>("service_name", "/led/set_animation"),
        BT::InputPort<unsigned>("id", "Animation ID to trigger."),
        BT::InputPort<std::string>("param", "Optional animation parameter."),
        BT::InputPort<bool>("repeating", "Specifies whether the animation should repeated continuously.")
    };
}

BT::NodeStatus CallSetLedAnimationService::tick()
{
    unsigned animation_id;
    
    if (!getInput<unsigned>("id", animation_id)) {
        return BT::NodeStatus::FAILURE;
    }

    request_->animation.id = static_cast<uint16_t>(animation_id);

    if (!getInput<std::string>("param", request_->animation.param)) {
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput<bool>("repeating", request_->repeating)) {
        return BT::NodeStatus::FAILURE;
    }

    service_client_->async_send_request(request_);
    
    return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_safety

#include <behaviortree_cpp/bt_factory.h>
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<rover_safety::CallSetLedAnimationService>("CallSetLedAnimationService");
}