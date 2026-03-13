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

#include "rover_safety/safety_node.hpp"

#include <algorithm>
#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

// Actions
#include "rover_safety/plugins/action/call_set_bool_service_node.hpp"
#include "rover_safety/plugins/action/call_trigger_service_node.hpp"
#include "rover_safety/plugins/action/execute_command_node.hpp"
#include "rover_safety/plugins/action/signal_shutdown_node.hpp"
// Decorators
#include "rover_safety/plugins/decorator/tick_after_timeout_node.hpp"

namespace rover_safety
{

using namespace std::chrono_literals;

SafetyNode::SafetyNode(
    const std::string & node_name, 
    const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
    RCLCPP_INFO(this->get_logger(), "Constructing node.");

    this->param_listener_ =
        std::make_shared<safety::ParamListener>(this->get_node_parameters_interface());
    this->params_ = this->param_listener_->get_params();

    RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

SafetyNode::~SafetyNode()
{

}

void SafetyNode::init()
{
    RCLCPP_INFO(this->get_logger(), "Initializing.");

    const auto bt_server_port = this->get_parameter("bt_server_port").as_int();
    const auto safety_initial_blackboard = createSafetyInitialBlackboard();
    
    safety_tree_ = std::make_unique<BehaviorTreeSafety>(
        this->shared_from_this(), "RoverSafety", safety_initial_blackboard, bt_server_port);
    registerBehaviorTree();
    safety_tree_->init(factory_);
    
    using namespace std::placeholders;

    battery_sub_ = this->create_subscription<BatteryStateMsg>(
        "rover_battery/battery_status", 10, 
        std::bind(&SafetyNode::batteryStateSubscriberCallback, this, _1));
    driver_state_sub_ = this->create_subscription<RoverDriverStateMsg>(
        "hardware_interface/rover_driver_state", 10, 
        std::bind(&SafetyNode::driverStateSubscriberCallback, this, _1));
    io_state_sub_ = this->create_subscription<IOStateMsg>(
        "hardware_interface/gpio_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&SafetyNode::ioStateSubscriberCallback, this, _1));
    system_status_sub_ = this->create_subscription<SystemStatusMsg>(
        "system_status", 10, 
        std::bind(&SafetyNode::systemStatusSubscriberCallback, this, _1));

    const double timer_freq = this->params_.timer_frequency;
    const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

    safety_tree_timer_ = this->create_wall_timer(
        timer_period, std::bind(&SafetyNode::safetyTreeTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void SafetyNode::registerBehaviorTree()
{
    const auto bt_project_path = this->params_.bt_project_path;

    // TODO: Register form config file
    // Actions
    factory_.registerNodeType<CallSetBoolService>("CallSetBoolService");
    factory_.registerNodeType<CallTriggerService>("CallTriggerService");
    factory_.registerNodeType<ExecuteCommand>("ExecuteCommand");
    factory_.registerNodeType<SignalShutdown>("SignalShutdown");
    // Decorators
    factory_.registerNodeType<SafetyBtTickAfterTimeout>("SafetyBtTickAfterTimeout");
    
    factory_.registerBehaviorTreeFromFile(bt_project_path);

    RCLCPP_INFO_STREAM(this->get_logger(), "BehaviorTree registered from path '" << bt_project_path << "'");
}

std::map<std::string, std::any> SafetyNode::createSafetyInitialBlackboard()
{
    const std::map<std::string, std::any> safety_initial_bb = {
        {"CRITICAL_BAT_TEMP", kCriticalBatteryTemp},
        {"FATAL_BAT_TEMP", kFatalBatteryTemp},
        {"POWER_SUPPLY_HEALTH_UNKNOWN", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN)},
        {"POWER_SUPPLY_HEALTH_GOOD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD)},
        {"POWER_SUPPLY_HEALTH_OVERHEAT", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT)},
        {"POWER_SUPPLY_HEALTH_DEAD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD)},
        {"POWER_SUPPLY_HEALTH_OVERVOLTAGE", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE)},
        {"POWER_SUPPLY_HEALTH_UNSPEC_FAILURE", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)},
        {"POWER_SUPPLY_HEALTH_COLD", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD)},
        {"POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE)},
        {"POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE)},
        {"battery_status", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN)},
        {"default_server_timeout", std::chrono::milliseconds(100)},
        {"bt_loop_duration", std::chrono::milliseconds(10)},
        {"wait_for_service_timeout", std::chrono::milliseconds(3000)},
    };

    RCLCPP_INFO(this->get_logger(), "Blackboard created.");

    return safety_initial_bb;
}

void SafetyNode::batteryStateSubscriberCallback(const BatteryStateMsg::SharedPtr battery_state)
{
    const auto battery_status = battery_state->power_supply_status;
    const auto battery_health = battery_state->power_supply_health;
 
    safety_tree_->getBlackboard()->set<unsigned>("battery_status", battery_status);
    safety_tree_->getBlackboard()->set<unsigned>("battery_health", battery_health);

    if (battery_status != BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN &&
        battery_health != BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
        battery_temp_ = battery_state->temperature;
    }

    safety_tree_->getBlackboard()->set<float>("bat_temp", battery_temp_);
}

void SafetyNode::driverStateSubscriberCallback(const RoverDriverStateMsg::SharedPtr driver_state)
{
    (void)driver_state;

    if (driver_state->driver_states.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty driver state message.");
        return;
    }

    // TODO: Implement driver temperature safety requirements
}

void SafetyNode::ioStateSubscriberCallback(const IOStateMsg::SharedPtr io_state)
{
    (void)io_state;

    // TODO: IO Safety Requirements
}

void SafetyNode::systemStatusSubscriberCallback(const SystemStatusMsg::SharedPtr system_status)
{
    (void)system_status;
    
    cpu_temp_ = system_status->cpu_temp;
    safety_tree_->getBlackboard()->set<float>("cpu_temp", cpu_temp_);
}

bool SafetyNode::systemReady()
{
    if (!safety_tree_->getBlackboard()->getEntry("battery_health") ||
        !safety_tree_->getBlackboard()->getEntry("battery_status") ||
        !safety_tree_->getBlackboard()->getEntry("bat_temp") ||
        !safety_tree_->getBlackboard()->getEntry("cpu_temp")) {

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, "Waiting for required system messages to arrive.");
    
        return false;
    }

    return true;
}

void SafetyNode::safetyTreeTimerCallback()
{
    if (!systemReady()) {
        return;
    }

    safety_tree_->tickOnce();

    if (safety_tree_->getTreeStatus() == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN(this->get_logger(), "Safety behavior tree returned FAILURE status");
    }
}

}  // namespace rover_safety