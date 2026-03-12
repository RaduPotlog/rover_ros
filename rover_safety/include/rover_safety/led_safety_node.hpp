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

#ifndef ROVER_SAFETY_LED_SAFETY_NODE_HPP_
#define ROVER_SAFETY_LED_SAFETY_NODE_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include <behaviortree_cpp/behavior_tree.h>
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rover_msgs/msg/led_animation.hpp"
#include "rover_msgs/msg/gpio_state.hpp"

#include "rover_safety/behavior_tree.hpp"
#include "rover_safety/led_safety_parameters.hpp"

namespace rover_safety
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using BoolMsg = std_msgs::msg::Bool;
using LedAnimationMsg = rover_msgs::msg::LedAnimation;
using JoyMsg = sensor_msgs::msg::Joy;
using GpioMsg = rover_msgs::msg::GpioState; 

class LedSafetyNode : public rclcpp::Node
{

public:
  
    LedSafetyNode(
        const std::string & node_name, 
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    ~LedSafetyNode() 
    {

    }

    void init();

protected:

    void declareParameters();

    void registerBehaviorTree();

    std::map<std::string, std::any> createLedInitialBlackboard();

    bool systemReady();

    std::unique_ptr<BehaviorTreeSafety> led_tree_;

private:
  
    void batteryCallback(const BatteryStateMsg::SharedPtr battery);
    
    void gpioCallback(const GpioMsg::SharedPtr gpio_state);
    
    void joyCallback(const JoyMsg::SharedPtr joy);
    
    void ledTreeTimerCallback();

    static constexpr std::size_t kDeadManButtonIndex = 4;

    float update_charging_anim_step_;

    std::shared_ptr<led_safety::ParamListener> param_listener_;
    led_safety::Params params_;

    rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
    rclcpp::Subscription<GpioMsg>::SharedPtr gpio_sub_;
    rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr led_tree_timer_;

    double battery_percent_;
    
    BT::BehaviorTreeFactory factory_;
};

}  // namespace rover_safety

#endif  // ROVER_SAFETY_LED_SAFETY_NODE_HPP_
