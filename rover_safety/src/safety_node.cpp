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
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_safety/domain/safety_health.hpp"
#include "rover_safety/infrastructure/safety_diagnostics.hpp"

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

namespace
{

domain::BatteryHealth toBatteryHealth(std::uint8_t power_supply_health)
{
    using domain::BatteryHealth;

    switch (power_supply_health) {
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD: return BatteryHealth::Good;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT: return BatteryHealth::Overheat;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD: return BatteryHealth::Dead;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE: return BatteryHealth::Overvoltage;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: return BatteryHealth::UnspecFailure;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD: return BatteryHealth::Cold;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE:
            return BatteryHealth::WatchdogTimerExpire;
        case BatteryStateMsg::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE:
            return BatteryHealth::SafetyTimerExpire;
        default: return BatteryHealth::Unknown;
    }
}

}  // namespace

SafetyNode::SafetyNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
: nav2::LifecycleNode(node_name, options)
, param_listener_(std::make_shared<safety::ParamListener>(this->get_node_parameters_interface()))
, params_(param_listener_->get_params())
, battery_thresholds_(params_.battery.temp.critical, params_.battery.temp.fatal)
{
    // Created here, not in on_configure: diagnostics must report an unconfigured node too, and a
    // re-configure must not declare diagnostic_updater.period twice.
    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("Rover Safety");
    diagnostic_updater_->add("Safety inputs", this, &SafetyNode::diagnoseInputs);
    diagnostic_updater_->add("Battery safety verdict", this, &SafetyNode::diagnoseBatteryVerdict);
    diagnostic_updater_->add("Safety behavior tree", this, &SafetyNode::diagnoseBehaviorTree);

    RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

nav2::CallbackReturn SafetyNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    init();
    return nav2::CallbackReturn::SUCCESS;
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

    // Only the latest reading matters for the safety decision. Publishers are volatile,
    // so transient_local here would be QoS-incompatible.
    const auto latest_state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    battery_sub_ = rclcpp_lifecycle::LifecycleNode::create_subscription<BatteryStateMsg>(
        "rover_battery/battery_status", latest_state_qos,
        std::bind(&SafetyNode::batteryStateSubscriberCallback, this, _1));
    driver_state_sub_ = rclcpp_lifecycle::LifecycleNode::create_subscription<RoverDriverStateMsg>(
        "hardware_interface/rover_driver_state", 10,
        std::bind(&SafetyNode::driverStateSubscriberCallback, this, _1));
    io_state_sub_ = rclcpp_lifecycle::LifecycleNode::create_subscription<IOStateMsg>(
        "hardware_interface/gpio_state",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&SafetyNode::ioStateSubscriberCallback, this, _1));
    system_status_sub_ = rclcpp_lifecycle::LifecycleNode::create_subscription<SystemStatusMsg>(
        "system_status", latest_state_qos,
        std::bind(&SafetyNode::systemStatusSubscriberCallback, this, _1));

    const double timer_freq = this->params_.timer_frequency;
    const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

    safety_tree_timer_ = this->create_wall_timer(
        timer_period, std::bind(&SafetyNode::safetyTreeTimerCallback, this));

    configured_ = true;
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
    const auto server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(params_.ros_communication_timeout.response));

    const std::map<std::string, std::any> safety_initial_bb = {
        {"VERDICT_NONE", unsigned(domain::SafetyVerdict::None)},
        {"VERDICT_TRIP_E_STOP", unsigned(domain::SafetyVerdict::TripEStop)},
        {"VERDICT_SHUTDOWN", unsigned(domain::SafetyVerdict::Shutdown)},
        {"battery_status", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN)},
        // Read by nav2_behavior_tree::BtServiceNode; must exceed the tick period so a
        // response arriving between ticks is still collected.
        {"server_timeout", server_timeout},
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

    const auto decision = domain::evaluateBatterySafety(
        toBatteryHealth(battery_health), battery_temp_, battery_thresholds_);

    safety_tree_->getBlackboard()->set<unsigned>("battery_verdict", unsigned(decision.verdict));
    safety_tree_->getBlackboard()->set<std::string>("battery_verdict_reason", decision.reason);

    last_battery_stamp_ = std::chrono::steady_clock::now();
    last_battery_decision_ = decision;
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
    safety_tree_->getBlackboard()->set<bool>("sw_e_stop_state", io_state->gpio_pin_sw_e_stop_user_button);
    last_gpio_stamp_ = std::chrono::steady_clock::now();
}

void SafetyNode::systemStatusSubscriberCallback(const SystemStatusMsg::SharedPtr system_status)
{
    (void)system_status;
    
    cpu_temp_ = system_status->cpu_temp;
    safety_tree_->getBlackboard()->set<float>("cpu_temp", cpu_temp_);
    last_system_status_stamp_ = std::chrono::steady_clock::now();
}

bool SafetyNode::systemReady()
{
    if (!safety_tree_->getBlackboard()->getEntry("battery_verdict") ||  \
        !safety_tree_->getBlackboard()->getEntry("battery_status")  ||  \
        !safety_tree_->getBlackboard()->getEntry("cpu_temp")        ||  \
        !safety_tree_->getBlackboard()->getEntry("sw_e_stop_state")) {

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, "Waiting for required system messages to arrive.");
    
        return false;
    }

    return true;
}

void SafetyNode::safetyTreeTimerCallback()
{
    system_ready_ = systemReady();

    if (!system_ready_) {
        return;
    }

    safety_tree_->tickOnce();

    if (safety_tree_->getTreeStatus() == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN(this->get_logger(), "Safety behavior tree returned FAILURE status");
    }
}

void SafetyNode::diagnoseInputs(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    const auto now = std::chrono::steady_clock::now();
    const double timeout = param_listener_->get_params().input_timeout;

    // gpio_state is transient-local and published on change only, so it cannot go stale.
    infrastructure::fillSafetyInputsStatus(
        {
            {"rover_battery/battery_status", infrastructure::ageSeconds(last_battery_stamp_, now), timeout},
            {"system_status", infrastructure::ageSeconds(last_system_status_stamp_, now), timeout},
            {"hardware_interface/gpio_state", infrastructure::ageSeconds(last_gpio_stamp_, now), std::nullopt},
        },
        status);
}

void SafetyNode::diagnoseBatteryVerdict(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (!last_battery_decision_.has_value()) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No battery reading yet.");
        return;
    }

    const auto & decision = *last_battery_decision_;

    status.add("Verdict", domain::toString(decision.verdict));
    status.add("Battery temperature (C)", battery_temp_);
    status.add("Battery critical temperature (C)", battery_thresholds_.criticalTemp());
    status.add("Battery fatal temperature (C)", battery_thresholds_.fatalTemp());
    status.add("CPU temperature (C)", cpu_temp_);

    status.summary(
        infrastructure::toDiagnosticLevel(domain::verdictHealthLevel(decision.verdict)),
        decision.verdict == domain::SafetyVerdict::None ?
            std::string("Battery within safety limits.") : decision.reason);
}

void SafetyNode::diagnoseBehaviorTree(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    status.add("Lifecycle state", get_current_state().label());

    infrastructure::fillBehaviorTreeStatus(
        configured_, system_ready_,
        configured_ ? safety_tree_->getTreeStatus() : BT::NodeStatus::IDLE, status);
}

}  // namespace rover_safety
