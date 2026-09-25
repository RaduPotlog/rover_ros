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
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_safety/behavior_tree_utils.hpp"
#include "rover_safety/domain/safety_health.hpp"
#include "rover_safety/infrastructure/battery_state_conversion.hpp"
#include "rover_safety/infrastructure/safety_diagnostics.hpp"
#include "rover_safety/infrastructure/shutdown_command.hpp"

namespace rover_safety
{

using namespace std::chrono_literals;

SafetyNode::SafetyNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
: nav2::LifecycleNode(node_name, options)
, param_listener_(std::make_shared<safety::ParamListener>(this->get_node_parameters_interface()))
, params_(param_listener_->get_params())
, battery_thresholds_(params_.battery.temp.critical, params_.battery.temp.fatal)
, shutdown_sequence_(std::chrono::duration<double>(params_.shutdown.retry_backoff))
{
    configure_retry_ = std::make_unique<infrastructure::ConfigureRetry>(
        *this, std::chrono::duration<double>(params_.configure_retry_period));

    // Created here, not in on_configure: diagnostics must report an unconfigured node too, and a
    // re-configure must not declare diagnostic_updater.period twice.
    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("Rover Safety");
    diagnostic_updater_->add("Safety inputs", this, &SafetyNode::diagnoseInputs);
    diagnostic_updater_->add("Battery safety verdict", this, &SafetyNode::diagnoseBatteryVerdict);
    diagnostic_updater_->add("Safety behavior tree", this, &SafetyNode::diagnoseBehaviorTree);
    diagnostic_updater_->add("Shutdown", this, &SafetyNode::diagnoseShutdown);

    RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

nav2::CallbackReturn SafetyNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (configured_) {
        return nav2::CallbackReturn::SUCCESS;
    }

    try {
        init();
    } catch (const std::exception & e) {
        // A missing plugin library, a bad tree project or an unavailable service throws while the
        // trees are built. Stay unconfigured so the transition can be retried.
        RCLCPP_ERROR(this->get_logger(), "Configuration failed: %s", e.what());
        safety_tree_timer_.reset();
        shutdown_service_.reset();
        configure_retry_->onFailure(e.what());
        return nav2::CallbackReturn::FAILURE;
    }

    configure_retry_->onSuccess();
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

    // A retried configure needs a fresh factory, since plugins and trees register only once. It is
    // recreated, not move-assigned: a move-assigned BT::BehaviorTreeFactory (BT.CPP 4.10) crashes
    // while parsing tree files.
    shutdown_tree_.reset();
    safety_tree_.reset();
    factory_ = std::make_unique<BT::BehaviorTreeFactory>();
    safety_tree_ = std::make_unique<BehaviorTreeSafety>(
        this->shared_from_this(), "RoverSafety", safety_initial_blackboard, bt_server_port);
    registerBehaviorTree();
    safety_tree_->init(*factory_);

    shutdown_command_ = resolveShutdownCommand();
    shutdown_tree_ = std::make_unique<BehaviorTreeSafety>(
        this->shared_from_this(), "RoverShutdown", createShutdownInitialBlackboard(),
        params_.shutdown.bt_server_port);
    shutdown_tree_->init(*factory_);

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
        "hardware_interface/safety_command_echo",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        std::bind(&SafetyNode::ioStateSubscriberCallback, this, _1));
    system_status_sub_ = rclcpp_lifecycle::LifecycleNode::create_subscription<SystemStatusMsg>(
        "system_status", latest_state_qos,
        std::bind(&SafetyNode::systemStatusSubscriberCallback, this, _1));

    const double timer_freq = this->params_.timer_frequency;
    const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

    safety_tree_timer_ = this->create_wall_timer(
        timer_period, std::bind(&SafetyNode::safetyTreeTimerCallback, this));

    if (params_.shutdown.service_enabled) {
        shutdown_service_ = this->create_service<TriggerSrv>(
            "~/shutdown", std::bind(&SafetyNode::shutdownServiceCallback, this, _1, _2, _3));
    }

    RCLCPP_INFO_STREAM(
        this->get_logger(), "Shutdown command: '" << shutdown_command_ << "'; service ~/shutdown "
        << (params_.shutdown.service_enabled ? "enabled" : "disabled") << ".");

    configured_ = true;
    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void SafetyNode::registerBehaviorTree()
{
    const auto bt_project_path = this->params_.bt_project_path;

    rover_safety::registerBehaviorTree(
        *factory_, bt_project_path, params_.plugin_libs, params_.ros_plugin_libs);

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

std::map<std::string, std::any> SafetyNode::createShutdownInitialBlackboard()
{
    const auto server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(params_.ros_communication_timeout.response));

    return {
        {"SHUTDOWN_HOSTS_FILE", params_.shutdown_hosts_path},
        {"SHUTDOWN_COMMAND_TIMEOUT", static_cast<float>(params_.shutdown.command_timeout)},
        // Replaced with the reason by requestShutdown(); never run as is.
        {"SHUTDOWN_LOCALHOST_COMMAND",
            infrastructure::buildShutdownCommand(shutdown_command_, "unknown")},
        {"server_timeout", server_timeout},
        {"bt_loop_duration", std::chrono::milliseconds(10)},
        {"wait_for_service_timeout", std::chrono::milliseconds(3000)},
    };
}

std::string SafetyNode::resolveShutdownCommand() const
{
    if (!params_.shutdown.command.empty()) {
        return params_.shutdown.command;
    }

    std::filesystem::path prefix;
    ament_index_cpp::get_package_prefix("rover_safety", prefix);

    // The tree trips the E-Stop itself before powering off.
    const auto script = prefix / "lib" / "rover_safety" / kShutdownScript;
    return infrastructure::shellQuote(script.string()) + " --no-e-stop";
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
        infrastructure::toBatteryHealth(battery_health), battery_temp_, battery_thresholds_);

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
    safety_tree_->getBlackboard()->set<bool>("sw_e_stop_state", io_state->sw_e_stop_user_button);
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
    switch (shutdown_sequence_.state()) {
        case domain::ShutdownState::InProgress:
            tickShutdownTree();
            return;
        case domain::ShutdownState::Succeeded:
            // Powering off: the safety tree stays halted.
            return;
        case domain::ShutdownState::Idle:
        case domain::ShutdownState::Failed:
            break;
    }

    system_ready_ = systemReady();

    if (!system_ready_) {
        return;
    }

    safety_tree_->tickOnce();

    if (safety_tree_->getTreeStatus() == BT::NodeStatus::FAILURE) {
        RCLCPP_WARN(this->get_logger(), "Safety behavior tree returned FAILURE status");
    }

    consumeShutdownSignal();
}

void SafetyNode::consumeShutdownSignal()
{
    std::pair<bool, std::string> signal_shutdown;

    if (!safety_tree_->getBlackboard()->get<std::pair<bool, std::string>>("signal_shutdown", signal_shutdown) ||
        !signal_shutdown.first) {
        return;
    }

    // Consumed: SignalShutdown must fire again to request another attempt.
    safety_tree_->getBlackboard()->set<std::pair<bool, std::string>>(
        "signal_shutdown", std::make_pair(false, std::string()));

    requestShutdown(
        signal_shutdown.second.empty() ? std::string("Requested by the safety behavior tree.") :
                                         signal_shutdown.second);
}

domain::ShutdownRequestResult SafetyNode::requestShutdown(const std::string & reason)
{
    const auto result = shutdown_sequence_.request(reason, std::chrono::steady_clock::now());

    if (result != domain::ShutdownRequestResult::Started) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, "Shutdown request refused (%s): %s",
            reason.c_str(), domain::toString(result));
        return result;
    }

    RCLCPP_WARN(
        this->get_logger(), "Shutting down the ROS controller (attempt %u): %s",
        shutdown_sequence_.attempts(), reason.c_str());

    safety_tree_->haltTree();
    shutdown_tree_->haltTree();
    shutdown_tree_->getBlackboard()->set<std::string>(
        "SHUTDOWN_LOCALHOST_COMMAND", infrastructure::buildShutdownCommand(shutdown_command_, reason));

    return result;
}

void SafetyNode::tickShutdownTree()
{
    shutdown_tree_->tickOnce();

    const auto status = shutdown_tree_->getTreeStatus();

    if (status == BT::NodeStatus::RUNNING) {
        return;
    }

    const bool succeeded = status == BT::NodeStatus::SUCCESS;

    shutdown_sequence_.finish(
        succeeded, succeeded ? std::string() : "Power-off command failed; see the rover_safety_node log.",
        std::chrono::steady_clock::now());

    if (succeeded) {
        RCLCPP_WARN(this->get_logger(), "Power-off requested; the ROS controller is going down.");
    } else {
        RCLCPP_ERROR(
            this->get_logger(), "Shutdown failed. A new request is accepted after %.1f s.",
            params_.shutdown.retry_backoff);
    }
}

void SafetyNode::shutdownServiceCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<TriggerSrv::Request> /*request*/,
    std::shared_ptr<TriggerSrv::Response> response)
{
    const auto result = requestShutdown("Requested via the ~/shutdown service.");

    response->success = result == domain::ShutdownRequestResult::Started;
    response->message = domain::toString(result);
}

void SafetyNode::diagnoseInputs(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    const auto now = std::chrono::steady_clock::now();
    const double timeout = param_listener_->get_params().input_timeout;

    // safety_command_echo is graded against input_timeout like every other input. An earlier version
    // passed std::nullopt here on the premise that the topic is "transient-local and published on
    // change only, so it cannot go stale". That premise is wrong: RoverSystem publishes it
    // periodically from read(), gated on driver_states_update_period_ (20 Hz), not on change. The
    // transient-local durability only means a late joiner gets the last sample - it says nothing
    // about the publisher still being alive. Leaving it ungraded meant a dead hardware interface
    // showed up here as a large, quietly-unjudged age value.
    infrastructure::fillSafetyInputsStatus(
        configured_,
        {
            {"rover_battery/battery_status", infrastructure::ageSeconds(last_battery_stamp_, now), timeout},
            {"system_status", infrastructure::ageSeconds(last_system_status_stamp_, now), timeout},
            {"hardware_interface/safety_command_echo", infrastructure::ageSeconds(last_gpio_stamp_, now), timeout},
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

void SafetyNode::diagnoseShutdown(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    infrastructure::fillShutdownStatus(shutdown_sequence_, status);
}

void SafetyNode::diagnoseBehaviorTree(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    status.add("Lifecycle state", get_current_state().label());

    infrastructure::fillBehaviorTreeStatus(
        configured_, system_ready_,
        configured_ ? safety_tree_->getTreeStatus() : BT::NodeStatus::IDLE,
        configure_retry_->failedAttempts(), configure_retry_->lastError(), status);
}

}  // namespace rover_safety
