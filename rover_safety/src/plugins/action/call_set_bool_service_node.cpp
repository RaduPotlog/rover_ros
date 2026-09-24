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

#include "rover_safety/plugins/action/call_set_bool_service_node.hpp"

namespace rover_safety
{

CallSetBoolService::CallSetBoolService(
    const std::string& name, 
    const BT::NodeConfig& config,
    const std::string & service_name)
: nav2_behavior_tree::BtServiceNode<std_srvs::srv::SetBool>(name, config, service_name)
{
}

BT::PortsList CallSetBoolService::providedPorts()
{
    return providedBasicPorts({ BT::InputPort<bool>("data") });
}

void CallSetBoolService::on_tick()
{
    if (!getInput<bool>("data", request_->data)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [data] for %s", service_name_.c_str());
        should_send_request_ = false;
    }
}

BT::NodeStatus CallSetBoolService::on_completion(
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
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
    factory.registerNodeType<rover_safety::CallSetBoolService>("CallSetBoolService");
}