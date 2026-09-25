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

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include "rover_safety/behavior_tree.hpp"
#include "rover_safety/behavior_tree_utils.hpp"
#include "rover_safety/led_safety_parameters.hpp"
#include "rover_safety/infrastructure/battery_state_conversion.hpp"
#include "rover_safety/infrastructure/safety_diagnostics.hpp"

namespace rover_safety
{

LedSafetyNode::LedSafetyNode(
    const std::string & node_name, const rclcpp::NodeOptions & options)
: nav2::LifecycleNode(node_name, options)
{
    RCLCPP_INFO(this->get_logger(), "Constructing node.");

    this->param_listener_ =std::make_shared<led_safety::ParamListener>(this->get_node_parameters_interface());
    this->params_ = this->param_listener_->get_params();

    configure_retry_ = std::make_unique<infrastructure::ConfigureRetry>(
        *this, std::chrono::duration<double>(params_.configure_retry_period));

    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("Bumper Led");
    diagnostic_updater_->add("LED safety inputs", this, &LedSafetyNode::diagnoseInputs);
    diagnostic_updater_->add("LED safety behavior tree", this, &LedSafetyNode::diagnoseBehaviorTree);

    RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

nav2::CallbackReturn LedSafetyNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (configured_) {
        return nav2::CallbackReturn::SUCCESS;
    }

    try {
        init();
    } catch (const std::exception & e) {
        // A missing plugin library, a bad tree project or an unavailable service throws while the
        // tree is built. Stay unconfigured so the transition can be retried.
        RCLCPP_ERROR(this->get_logger(), "Configuration failed: %s", e.what());
        led_tree_timer_.reset();
        configure_retry_->onFailure(e.what());
        return nav2::CallbackReturn::FAILURE;
    }

    configure_retry_->onSuccess();
    return nav2::CallbackReturn::SUCCESS;
}

void LedSafetyNode::init()
{
    RCLCPP_INFO(this->get_logger(), "Initializing.");

    const auto bt_server_port = this->get_parameter("bt_server_port").as_int();
    const auto initial_blackboard = createLedInitialBlackboard();
    
    // A retried configure needs a fresh factory, since plugins and trees register only once. It is
    // recreated, not move-assigned: a move-assigned BT::BehaviorTreeFactory (BT.CPP 4.10) crashes
    // while parsing tree files.
    led_tree_.reset();
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();
    led_tree_ = std::make_unique<BehaviorTreeSafety>(
        this->shared_from_this(), "RoverLedSafety", initial_blackboard, bt_server_port);
    registerBehaviorTree();
    led_tree_->init(*factory_);

    using namespace std::placeholders;

    battery_sub_ = rclcpp_lifecycle::LifecycleNode::create_subscription<BatteryStateMsg>(
        "rover_battery/battery_status", 10,
        std::bind(&LedSafetyNode::batteryCallback, this, _1));
    
    gpio_sub_ = rclcpp_lifecycle::LifecycleNode::create_subscription<GpioMsg>(
        "hardware_interface/safety_status",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        std::bind(&LedSafetyNode::gpioCallback, this, _1));
    
    joy_sub_ = rclcpp_lifecycle::LifecycleNode::create_subscription<JoyMsg>(
        "joy", 10, std::bind(&LedSafetyNode::joyCallback, this, _1));

    const double timer_freq = this->params_.timer_frequency;
    const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

    led_tree_timer_ = this->create_wall_timer(
        timer_period, std::bind(&LedSafetyNode::ledTreeTimerCallback, this));

    configured_ = true;
    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void LedSafetyNode::registerBehaviorTree()
{
    const auto bt_project_path = this->params_.bt_project_path;

    rover_safety::registerBehaviorTree(
        *factory_, bt_project_path, params_.plugin_libs, params_.ros_plugin_libs);

    RCLCPP_INFO_STREAM(this->get_logger(), "BehaviorTree registered from path '" << bt_project_path << "'");
}

std::map<std::string, std::any> LedSafetyNode::createLedInitialBlackboard()
{
    const float critical_battery_threshold_percent =
        static_cast<float>(this->params_.battery.percent.threshold.critical);
    
    const float low_battery_anim_period = static_cast<float>(this->params_.battery.anim_period.low);
    
    const float low_battery_threshold_percent =
        static_cast<float>(this->params_.battery.percent.threshold.low);

    led_thresholds_ = {
        critical_battery_threshold_percent, low_battery_threshold_percent,
        static_cast<float>(this->params_.battery.charging_anim_step)};

    const auto server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(params_.ros_communication_timeout.response));
    const auto wait_for_service_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(params_.ros_communication_timeout.availability));

    const std::string undefined_charging_anim_percent = "";
    const int undefined_anim_id = -1;

    const std::map<std::string, std::any> led_initial_bb = {
        {"charging_anim_percent", undefined_charging_anim_percent},
        {"current_anim_id", undefined_anim_id},
        {"current_battery_anim_id", undefined_anim_id},
        {"current_error_anim_id", undefined_anim_id},
        {"drive_state", false},
        {"LOW_BATTERY_ANIM_PERIOD", low_battery_anim_period},
        // Verdicts of domain::evaluateLedAnimation(), which the shipped tree dispatches on.
        {"LED_STATE_READY", unsigned(domain::LedStateVerdict::Ready)},
        {"LED_STATE_MANUAL_ACTION", unsigned(domain::LedStateVerdict::ManualAction)},
        {"LED_STATE_E_STOP", unsigned(domain::LedStateVerdict::EStop)},
        {"LED_ERROR_NONE", unsigned(domain::LedErrorVerdict::None)},
        {"LED_ERROR_CHARGING_OVERHEAT", unsigned(domain::LedErrorVerdict::ChargingOverheat)},
        {"LED_ERROR_STATUS_UNKNOWN", unsigned(domain::LedErrorVerdict::StatusUnknown)},
        {"LED_BATTERY_NONE", unsigned(domain::LedBatteryVerdict::None)},
        {"LED_BATTERY_CHARGING", unsigned(domain::LedBatteryVerdict::Charging)},
        {"LED_BATTERY_DISCHARGING", unsigned(domain::LedBatteryVerdict::Discharging)},
        // Battery thresholds, status and health constants: no longer read by the shipped tree;
        // kept for trees supplied through led_bt_project_path.
        {"CRITICAL_BATTERY_THRESHOLD_PERCENT", critical_battery_threshold_percent},
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
        // Battery status constants (see above)
        {"POWER_SUPPLY_STATUS_UNKNOWN", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN)},
        {"POWER_SUPPLY_STATUS_CHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING)},
        {"POWER_SUPPLY_STATUS_DISCHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING)},
        {"POWER_SUPPLY_STATUS_NOT_CHARGING", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING)},
        {"POWER_SUPPLY_STATUS_FULL", unsigned(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL)},
        // Battery health constants (see above)
        {"POWER_SUPPLY_HEALTH_OVERHEAT", unsigned(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT)},
        // Behaviour tree constants, read by nav2_behavior_tree::BtServiceNode. server_timeout must
        // exceed the tick period so a response arriving between ticks is still collected.
        // bt_loop_duration bounds how long each service node spins per tick; the tree runs
        // several of them in parallel, so it stays short.
        {"server_timeout", server_timeout},
        {"bt_loop_duration", std::chrono::milliseconds(10)},
        {"wait_for_service_timeout", wait_for_service_timeout},
    };

    RCLCPP_INFO(this->get_logger(), "Blackboard created.");
  
    return led_initial_bb;
}

void LedSafetyNode::batteryCallback(const BatteryStateMsg::SharedPtr battery_state)
{
    const auto battery_status = battery_state->power_supply_status;
    const auto battery_health = battery_state->power_supply_health;

    led_tree_->getBlackboard()->set<unsigned>("battery_status", battery_status);
    led_tree_->getBlackboard()->set<unsigned>("battery_health", battery_health);

    if (battery_status != BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN &&
        battery_health != BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
        led_inputs_.battery_percent = battery_state->percentage;
    }

    led_inputs_.battery_status = infrastructure::toPowerSupplyStatus(battery_status);
    led_inputs_.battery_health = infrastructure::toBatteryHealth(battery_health);
    const auto decision = updateLedVerdicts();

    led_tree_->getBlackboard()->set<float>("battery_percent", led_inputs_.battery_percent);
    led_tree_->getBlackboard()->set<std::string>("battery_percent_round", decision.battery_percent_round);

    last_battery_stamp_ = std::chrono::steady_clock::now();
}

void LedSafetyNode::gpioCallback(const GpioMsg::SharedPtr gpio_state)
{
    led_tree_->getBlackboard()->set<bool>("e_stop_state", gpio_state->hw_e_stop_user_button);
    led_inputs_.e_stop_pressed = gpio_state->hw_e_stop_user_button;
    updateLedVerdicts();
    last_gpio_stamp_ = std::chrono::steady_clock::now();
}

void LedSafetyNode::joyCallback(const JoyMsg::SharedPtr joy)
{
    const bool held = joy->buttons[kDeadManButtonIndex];
    led_tree_->getBlackboard()->set<bool>("drive_state", held);
    led_inputs_.dead_man_held = held;
    updateLedVerdicts();
    last_joy_stamp_ = std::chrono::steady_clock::now();
}

domain::LedAnimationDecision LedSafetyNode::updateLedVerdicts()
{
    // Written next to the raw inputs, in the same callback, so every tick sees matching values.
    const auto decision = domain::evaluateLedAnimation(led_inputs_, led_thresholds_);
    const auto blackboard = led_tree_->getBlackboard();

    blackboard->set<unsigned>("led_state_verdict", unsigned(decision.state));
    blackboard->set<unsigned>("led_error_verdict", unsigned(decision.error));
    blackboard->set<unsigned>("led_battery_verdict", unsigned(decision.battery));
    blackboard->set<bool>("led_battery_full", decision.battery_full);
    blackboard->set<bool>("led_battery_low", decision.low_battery);
    blackboard->set<bool>("led_battery_critical", decision.critical_battery);
    blackboard->set<bool>("led_battery_nominal", decision.nominal_battery);

    return decision;
}

void LedSafetyNode::ledTreeTimerCallback()
{
    system_ready_ = systemReady();

    if (!system_ready_) {
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

void LedSafetyNode::diagnoseInputs(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    const auto now = std::chrono::steady_clock::now();
    const double timeout = param_listener_->get_params().input_timeout;

    // joy is optional for the tree (it only selects the drive animation), so it is reported as a
    // value but not graded.
    //
    // safety_status is graded against input_timeout like every other input. An earlier version
    // passed std::nullopt here on the premise that the topic is "transient-local and published on
    // change only, so it cannot go stale". That premise is wrong: RoverSystem publishes it
    // periodically from read(), gated on driver_states_update_period_ (20 Hz), not on change. The
    // transient-local durability only means a late joiner gets the last sample - it says nothing
    // about the publisher still being alive. Leaving it ungraded meant a dead hardware interface
    // showed up here as a large, quietly-unjudged age value.
    status.add("joy age (s)", last_joy_stamp_ ?
        std::to_string(*infrastructure::ageSeconds(last_joy_stamp_, now)) :
        std::string("never received"));
    status.add("Battery percent", led_inputs_.battery_percent);

    infrastructure::fillSafetyInputsStatus(
        configured_,
        {
            {"rover_battery/battery_status", infrastructure::ageSeconds(last_battery_stamp_, now), timeout},
            {"hardware_interface/safety_status", infrastructure::ageSeconds(last_gpio_stamp_, now), timeout},
        },
        status);
}

void LedSafetyNode::diagnoseBehaviorTree(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    status.add("Lifecycle state", get_current_state().label());

    infrastructure::fillBehaviorTreeStatus(
        configured_, system_ready_,
        configured_ ? led_tree_->getTreeStatus() : BT::NodeStatus::IDLE,
        configure_retry_->failedAttempts(), configure_retry_->lastError(), status);
}

}  // namespace rover_safety
