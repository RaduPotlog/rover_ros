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

#include "rover_safety/plugins/action/execute_command_node.hpp"

#include <string>

#include "behaviortree_cpp/exceptions.h"

#include "rover_safety/behavior_tree_utils.hpp"

namespace rover_safety
{

BT::NodeStatus ExecuteCommand::onStart()
{
    std::string command;
    
    if (!this->getInput<std::string>("command", command)) {
        RCLCPP_ERROR_STREAM(*logger_, getLoggerPrefix(name()) << "Failed to get input [command]");
        
        return BT::NodeStatus::FAILURE;
    }

    float timeout;
    
    if (!this->getInput<float>("timeout", timeout)) {
        RCLCPP_ERROR_STREAM(*logger_, getLoggerPrefix(name()) << "Failed to get input [timeout]");
    
        return BT::NodeStatus::FAILURE;
    }

    const auto timeout_ms = std::chrono::milliseconds(static_cast<long long>(timeout * 1000));
    command_handler_->execute(command, timeout_ms);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteCommand::onRunning()
{
    if (command_handler_->getState() == CommandState::RUNNING) {
        return BT::NodeStatus::RUNNING;
    }

    if (command_handler_->getState() == CommandState::SUCCESS) {
        RCLCPP_INFO_STREAM(
            *logger_, getLoggerPrefix(name()) << "Command output: " << command_handler_->getOutput());
        
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR_STREAM(
        *logger_, getLoggerPrefix(name()) << "Command failed: " << command_handler_->getError());
    
    RCLCPP_INFO_STREAM(
        *logger_, getLoggerPrefix(name()) << "Command output: " << command_handler_->getOutput());
    
    return BT::NodeStatus::FAILURE;
}

void ExecuteCommand::onHalted() 
{ 
    command_handler_->halt(); 
}

}  // namespace rover_safety

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rover_safety::ExecuteCommand>("ExecuteCommand");
}