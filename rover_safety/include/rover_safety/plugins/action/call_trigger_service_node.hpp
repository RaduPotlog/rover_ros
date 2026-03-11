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

#ifndef ROVER_SAFETY_PLUGINS_ACTION_CALL_TRIGGER_SERVICE_NODE_HPP_
#define ROVER_SAFETY_PLUGINS_ACTION_CALL_TRIGGER_SERVICE_NODE_HPP_

#include <string>

#include <behaviortree_cpp/behavior_tree.h>
#include <nav2_behavior_tree/bt_service_node.hpp>
#include "rclcpp/rclcpp.hpp"

#include <std_srvs/srv/trigger.hpp>

namespace rover_safety
{

class CallTriggerService : public nav2_behavior_tree::BtServiceNode<std_srvs::srv::Trigger>
{

public:

    CallTriggerService(
        const std::string& name, 
        const BT::NodeConfig & config,
        const std::string & service_name = "");

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:

    std::string service_name_;

    rclcpp::Node::SharedPtr node_;
};

}  // namespace rover_safety

#endif  // ROVER_SAFETY_PLUGINS_ACTION_CALL_TRIGGER_SERVICE_NODE_HPP_