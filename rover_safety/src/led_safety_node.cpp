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

#include "rover_safety/led_safety_node.hpp"

#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include "rover_safety/behavior_tree.hpp"
#include "rover_safety/behavior_tree_utils.hpp"
#include "rover_safety/led_safety_parameters.hpp"

// Actions
#include "rover_safety/plugins/action/call_set_led_animation_service_node.hpp"
// Decorators
#include "rover_safety/plugins/decorator/tick_after_timeout_node.hpp"

namespace rover_safety
{

LedSafetyNode::LedSafetyNode(
    const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
    RCLCPP_INFO(this->get_logger(), "Constructing node.");

    this->param_listener_ =std::make_shared<led_safety::ParamListener>(this->get_node_parameters_interface());
    this->params_ = this->param_listener_->get_params();

    battery_percent_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void LedSafetyNode::init()
{
    RCLCPP_INFO(this->get_logger(), "Initializing.");

    const auto bt_server_port = this->get_parameter("bt_server_port").as_int();
    const auto initial_blackboard = createLedInitialBlackboard();
    
    led_tree_ = std::make_unique<BehaviorTreeSafety>(
        this->shared_from_this(), "RoverLedSafety", initial_blackboard, bt_server_port);
    registerBehaviorTree();
    led_tree_->init(factory_);

    using namespace std::placeholders;

    battery_sub_ = this->create_subscription<BatteryStateMsg>(
        "rover_battery/battery_status", 10, std::bind(&LedSafetyNode::batteryCallback, this, _1));
    
    gpio_sub_ = this->create_subscription<GpioMsg>(
        "hardware_interface/gpio_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&LedSafetyNode::gpioCallback, this, _1));
    
    joy_sub_ = this->create_subscription<JoyMsg>(
        "joy", 10, std::bind(&LedSafetyNode::joyCallback, this, _1));

    const double timer_freq = this->params_.timer_frequency;
    const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

    led_tree_timer_ = this->create_wall_timer(
        timer_period, std::bind(&LedSafetyNode::ledTreeTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void LedSafetyNode::registerBehaviorTree()
{
    const auto bt_project_path = this->params_.bt_project_path;

    // TODO: Register form config file
    // Actions
    factory_.registerNodeType<CallSetLedAnimationService>("CallSetLedAnimationService");
    // Decorators
    factory_.registerNodeType<SafetyBtTickAfterTimeout>("SafetyBtTickAfterTimeout");
    
    factory_.registerBehaviorTreeFromFile(bt_project_path);

    RCLCPP_INFO_STREAM(this->get_logger(), "BehaviorTree registered from path '" << bt_project_path << "'");
}

std::map<std::string, std::any> LedSafetyNode::createLedInitialBlackboard()
{
    update_charging_anim_step_ = this->params_.battery.charging_anim_step;
    
    const float critical_battery_threshold_percent =
        static_cast<float>(this->params_.battery.percent.threshold.critical);
    
    const float low_battery_anim_period = static_cast<float>(this->params_.battery.anim_period.low);
    
    const float low_battery_threshold_percent =
        static_cast<float>(this->params_.battery.percent.threshold.low);

    const std::string undefined_charging_anim_percent = "";
    const int undefined_anim_id = -1;

    const std::map<std::string, std::any> led_initial_bb = {
        {"charging_anim_percent", undefined_charging_anim_percent},
        {"current_anim_id", undefined_anim_id},
        {"current_battery_anim_id", undefined_anim_id},
        {"current_error_anim_id", undefined_anim_id},
        {"drive_state", false},
        {"CRITICAL_BATTERY_THRESHOLD_PERCENT", critical_battery_threshold_percent},
        {"LOW_BATTERY_ANIM_PERIOD", low_battery_anim_period},
        {"LOW_BATTERY_THRESHOLD_PERCENT", low_battery_threshold_percent},
        // Animation images constants
        {"E_STOP_ANIM_ID", unsigned(LedAnimationMsg::E_STOP)},
        {"READY_ANIM_ID", unsigned(LedAnimationMsg::READY)},
        {"ERROR_ANIM_ID", unsigned(LedAnimationMsg::ERROR)},
        {"NO_ERROR_ANIM_ID", unsigned(LedAnimationMsg::NO_ERROR)},
        {"MANUAL_ACTION_ANIM_ID", unsigned(LedAnimationMsg::MANUAL_ACTION)},
        {"LOW_BATTERY_ANIM_ID", unsigned(LedAnimationMsg::LOW_BATTERY)},
        {"CRITICAL_BATTERY_ANIM_ID", unsigned(LedAnimationMsg::CRITICAL_BATTERY)},
        {"CHARGING_BATTERY_ANIM_ID", unsigned(LedAnimationMsg::CHARGING_BATTERY)},
        {"BATTERY_CHARGED_ANIM_ID", unsigned(LedAnimationMsg::BATTERY_CHARGED)},
        {"CHARGER_INSERTED_ANIM_ID", unsigned(LedAnimationMsg::CHARGER_INSERTED)},
        {"BATTERY_NOMINAL_ANIM_ID", unsigned(LedAnimationMsg::BATTERY_NOMINAL)},
        // Battery status constants
        {"POWER_SUPPLY_STATUS_UNKNOWN", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN)},
        {"POWER_SUPPLY_STATUS_CHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING)},
        {"POWER_SUPPLY_STATUS_DISCHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING)},
        {"POWER_SUPPLY_STATUS_NOT_CHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING)},
        {"POWER_SUPPLY_STATUS_FULL", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL)},
        // Battery health constants
        {"POWER_SUPPLY_HEALTH_OVERHEAT", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT)},
        // Behaviour tree constants
        {"default_server_timeout", std::chrono::milliseconds(5000)},
        {"bt_loop_duration", std::chrono::milliseconds(100)},
        {"wait_for_service_timeout", std::chrono::milliseconds(3000)},
    };

    RCLCPP_INFO(this->get_logger(), "Blackboard created.");
  
    return led_initial_bb;
}

void LedSafetyNode::batteryCallback(const BatteryStateMsg::SharedPtr battery_state)
{
    // const auto battery_status = battery_state->power_supply_status;
    // const auto battery_health = battery_state->power_supply_health;
    
    const auto battery_status = BatteryStateMsg::POWER_SUPPLY_STATUS_FULL;//battery_state->power_supply_status;
    const auto battery_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;//battery_state->power_supply_health;

    led_tree_->getBlackboard()->set<unsigned>("battery_status", battery_status);
    led_tree_->getBlackboard()->set<unsigned>("battery_health", battery_health);

    if (battery_status != BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN &&
        battery_health != BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
        battery_percent_ = battery_state->percentage;
    }

    led_tree_->getBlackboard()->set<float>("battery_percent", battery_percent_);
    led_tree_->getBlackboard()->set<std::string>("battery_percent_round",
        std::to_string(round(battery_percent_ / update_charging_anim_step_) * update_charging_anim_step_));
}

void LedSafetyNode::gpioCallback(const GpioMsg::SharedPtr gpio_state)
{
    led_tree_->getBlackboard()->set<bool>("e_stop_state", gpio_state->gpio_pin_hw_e_stop_user_button);
}

void LedSafetyNode::joyCallback(const JoyMsg::SharedPtr joy)
{
    led_tree_->getBlackboard()->set<bool>("drive_state", joy->buttons[kDeadManButtonIndex]);
}

void LedSafetyNode::ledTreeTimerCallback()
{
    if (!systemReady()) {
        return;
    }

    led_tree_->tickOnce();

    if (led_tree_->getTreeStatus() == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN(this->get_logger(), "Led behavior tree returned FAILURE status");
    }
}

bool LedSafetyNode::systemReady()
{
    if (!led_tree_->getBlackboard()->getEntry("e_stop_state") ||
        !led_tree_->getBlackboard()->getEntry("battery_status")) {
    
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Waiting for required system messages to arrive.");
        
        return false;
    }

    return true;
}

}  // namespace husarion_ugv_manager