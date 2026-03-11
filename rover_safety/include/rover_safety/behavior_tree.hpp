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

#ifndef ROVER_SAFETY_BEHAVIOR_TREE_HPP_
#define ROVER_SAFETY_BEHAVIOR_TREE_HPP_

#include <any>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "std_msgs/msg/bool.hpp"

#include "rover_utils/networking_utils.hpp"

namespace rover_safety
{

class BehaviorTreeSafety
{

public:
  
    BehaviorTreeSafety(
        const rclcpp::Node::SharedPtr & node,
        const std::string & tree_name, 
        const std::map<std::string, std::any> & initial_blackboard,
        const unsigned groot_port = 1667)
    : node_(std::move(node))
    , tree_name_(tree_name)
    , initial_blackboard_(initial_blackboard)
    , groot_port_(groot_port)
    , tree_status_(BT::NodeStatus::IDLE)
    {
  
    }

    ~BehaviorTreeSafety() 
    {

    }
    
    inline void init(
        BT::BehaviorTreeFactory & factory)
    {
        config_ = createBTConfig(initial_blackboard_);

        tree_ = factory.createTree(tree_name_, config_.blackboard);

        const auto max_port = 65535;
        
        while (!rover_utils::isPortAvailable(groot_port_)) {
        
            if (groot_port_ >= max_port) {
                throw std::runtime_error("No available port for Groot2 publisher.");
            }

            RCLCPP_WARN_STREAM(
                rclcpp::get_logger("BehaviorTreeSafety"),
                    "Port " << groot_port_ << " is not available. Trying next port.");

            groot_port_++;
        }

        RCLCPP_INFO_STREAM(
            rclcpp::get_logger("BehaviorTreeSafety"),
                "Groot2 publisher started on port " << groot_port_);

        groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, groot_port_);
    }

    void tickOnce() 
    { 
        tree_status_ = tree_.tickOnce(); 
    }
    
    void tickExactlyOnce() 
    { 
        tree_status_ = tree_.tickExactlyOnce(); 
    }
    
    void tickWhileRunning() 
    { 
        tree_status_ = tree_.tickWhileRunning(); 
    }
    
    void haltTree() 
    { 
        tree_.haltTree(); 
    }

    BT::NodeStatus getTreeStatus() const 
    { 
        return tree_status_; 
    }
    
    BT::Tree & getTree()
    { 
        return tree_; 
    }
    
    BT::Blackboard::Ptr getBlackboard() const 
    {
        return config_.blackboard; 
    }

protected:

    inline BT::NodeConfig createBTConfig(const std::map<std::string, std::any> & bb_values) const
    {
        BT::NodeConfig config;
        
        config.blackboard = BT::Blackboard::create();

        if (!node_) {
            throw BT::RuntimeError("No rclcpp::Node available");
        }

        config.blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
        
        for (auto & [name, value] : bb_values) {
            
            const std::type_info & type = value.type();
            
            if (type == typeid(bool)) {
                config.blackboard->set<bool>(name, std::any_cast<bool>(value));
            } else if (type == typeid(int)) {
                config.blackboard->set<int>(name, std::any_cast<int>(value));
            } else if (type == typeid(unsigned)) {
                config.blackboard->set<unsigned>(name, std::any_cast<unsigned>(value));
            } else if (type == typeid(float)) {
                config.blackboard->set<float>(name, std::any_cast<float>(value));
            } else if (type == typeid(double)) {
                config.blackboard->set<double>(name, std::any_cast<double>(value));
            } else if (type == typeid(const char *)) {
                config.blackboard->set<std::string>(name, std::any_cast<const char *>(value));
            } else if (type == typeid(std::string)) {
                config.blackboard->set<std::string>(name, std::any_cast<std::string>(value));
            } else if (type == typeid(std::chrono::milliseconds)) {
                config.blackboard->set<std::chrono::milliseconds>(name, std::any_cast<std::chrono::milliseconds>(value));
            } else {
                throw std::invalid_argument(
                    "Invalid type for blackboard entry." 
                    "Valid types are: bool, int, unsigned, float, double, "
                    "const char*, std::string");
            }
        }

        return config;
    }

private:

    rclcpp::Node::SharedPtr node_;
    
    const std::string tree_name_;
    const std::map<std::string, std::any> initial_blackboard_;
    unsigned groot_port_;

    BT::Tree tree_;
    BT::NodeStatus tree_status_;
    BT::NodeConfig config_;

    std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
};

}  // namespace rover_safety

#endif  // ROVER_SAFETY_BEHAVIOR_TREE_HPP_
