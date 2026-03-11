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

#ifndef ROVER_SAFETY_PLUGINS_ACTION_EXECUTE_COMMAND_NODE_HPP_
#define ROVER_SAFETY_PLUGINS_ACTION_EXECUTE_COMMAND_NODE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/logger.hpp"

#include "rover_safety/plugins/command_handler.hpp"

namespace rover_safety
{

class ExecuteCommand : public BT::StatefulActionNode
{
public:
        
    ExecuteCommand(
        const std::string & name, 
        const BT::NodeConfiguration & conf)
    : StatefulActionNode(name, conf)
    {
        command_handler_ = std::make_shared<CommandHandler>();
        logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(name));
    }

    ~ExecuteCommand() = default;

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("command", "Command to execute."),
            BT::InputPort<float>("timeout", "Command timeout in seconds."),
        };
    }

protected:
  
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    std::shared_ptr<rclcpp::Logger> logger_;
    std::shared_ptr<CommandHandler> command_handler_;
};

}  // namespace rover_safety

#endif  // ROVER_SAFETY_PLUGINS_ACTION_EXECUTE_COMMAND_NODE_HPP_
