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

#include "rover_hardware_interface/system_ros_interface/system_ros_interface.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "rover_msgs/msg/fault_flag.hpp"
#include "rover_msgs/msg/runtime_error.hpp"

namespace rover_hardware_interface
{

// A poll older than this makes SafetyStatus.link_healthy false. Generous against the default
// 100 ms poll period so ordinary scheduling jitter never flaps it, tight enough that a link that
// has actually stopped is called out well inside rover_twist_mux's 1.0 s motion-lock timeout.
constexpr uint64_t kSafetyLinkStalePollAgeMs = 500;


namespace
{

// driverNamesToString() builds a fresh std::string on every call; getDriverStateByName() is on
// the RT path (called every driver-state-update cycle), so look the name up in a table computed
// once instead.
const std::string & cachedDriverName(const DriverNames name)
{
    static const std::array<std::string, 4> kNames = {
        driverNamesToString(DriverNames::REAR_LEFT),
        driverNamesToString(DriverNames::REAR_RIGHT),
        driverNamesToString(DriverNames::FRONT_LEFT),
        driverNamesToString(DriverNames::FRONT_RIGHT),
    };

    return kNames.at(static_cast<std::size_t>(name));
}

// Both messages are built from FaultFlag/RuntimeError's own named bit accessors (not a
// re-decoded copy of the bitset), the same accessors getErrorMap() is built from internally, so
// the published fault/runtime-error topics can never disagree with what diagnoseErrors() reports
// for the same underlying flags. Deliberately allocation-free (no intermediate std::map): this
// runs on the RT thread via RoverSystem::read() -> updateDriverStateMsg() -> updateMsgErrorFlags().
rover_msgs::msg::FaultFlag toFaultFlagMsg(const FaultFlag & fault_flag)
{
    rover_msgs::msg::FaultFlag msg;
    msg.emergency_stop = fault_flag.isEmergencyStop();
    msg.motor_setup_fault = fault_flag.isMotorSetupFault();
    return msg;
}

rover_msgs::msg::RuntimeError toRuntimeErrorMsg(const RuntimeError & runtime_error)
{
    rover_msgs::msg::RuntimeError msg;
    msg.safety_stop_active = runtime_error.isSafetyStopActive();
    return msg;
}

}  // namespace

// Trigger is the only service type this component exposes. A SetBool specialisation and its
// explicit instantiation used to sit here too, but nothing ever registered a SetBool service -
// dead code that read like a supported path. That is not an oversight to be corrected: the safety
// commands are deliberately three named Triggers (sw_user_e_stop_set / sw_user_e_stop_reset /
// sw_e_stop_latch_reset) rather than one parameterised setter, because a named service is what
// makes the intent and the authorisation legible at the call site and in a log.
template class ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>;

template <typename SrvT, typename CallbackT>
void ROSServiceWrapper<SrvT, CallbackT>::registerService(
    const rclcpp::Node::SharedPtr node, const std::string & service_name,
    rclcpp::CallbackGroup::SharedPtr group, const rclcpp::QoS & qos)
{
    service_ = node->create_service<SrvT>(
        service_name,
        std::bind(&ROSServiceWrapper<SrvT, CallbackT>::callbackWrapper, this,
        std::placeholders::_1, std::placeholders::_2),
        qos, group);
}

template <typename SrvT, typename CallbackT>
void ROSServiceWrapper<SrvT, CallbackT>::callbackWrapper(SrvRequestConstPtr request, SrvResponsePtr response)
{
    try {
        proccessCallback(request);
        response->success = true;
    } catch (const std::exception & err) {
        response->success = false;
        response->message = err.what();

        RCLCPP_WARN_STREAM(
            rclcpp::get_logger("ROSServiceWrapper"),
            "An exception occurred while handling the request: " << err.what());
    }
}

template <>
void ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>::proccessCallback(SrvRequestConstPtr /* request */)
{
    callback_();
}

SystemROSInterface::SystemROSInterface(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: node_(rclcpp::Node::make_shared(node_name, node_options)), diagnostic_updater_(node_)
{
    RCLCPP_INFO(rclcpp::get_logger("SystemROSInterface"), "Constructing node.");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);

    executor_thread_ = std::thread([this]() {
        executor_->spin(); }
    );

    // Periodic, non-latched telemetry: explicit reliable/volatile QoS (matches
    // ros2_communication.md's "Commands"/high-frequency-state guidance) rather than relying on
    // the create_publisher(size_t) default. Reliable (not best-effort sensor-style QoS) because
    // this is low-rate, gated by driver_states_update_period_ (RoverSystem::read()) rather than
    // ticking every RT cycle, so a dropped sample isn't self-correcting the way a high-frequency
    // sensor stream is; depth 5 gives a late-joining subscriber (e.g. a UI reconnecting) a couple
    // of cycles of buffered history without holding an unbounded backlog.
    driver_state_publisher_ = node_->create_publisher<RoverDriverStateMsg>(
        "hardware_interface/rover_driver_state",
        rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile());
    realtime_driver_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<RoverDriverStateMsg>>(driver_state_publisher_);

    // Pre-populate all 4 driver-state slots up front so getDriverStateByName() never has to
    // push_back() on the RT thread (RoverSystem::read() -> updateDriverStateMsg()) the first time
    // each DriverNames value is looked up.
    for (const auto name :
         {DriverNames::REAR_LEFT, DriverNames::REAR_RIGHT, DriverNames::FRONT_LEFT,
          DriverNames::FRONT_RIGHT}) {
        DriverStateNamedMsg driver_state_named;
        driver_state_named.name = cachedDriverName(name);
        driver_state_msg_.driver_states.push_back(driver_state_named);
    }

    // Volatile, not transient_local. The old gpio_state topic was latched, which is durability
    // meant for configuration that is published once; this is a periodic 20 Hz status stream. The
    // combination actively misled two consumers into commenting that the topic "cannot go stale"
    // and skipping their staleness checks - a latched sample tells a late joiner what was true
    // when the publisher last ran, not that it is still running. Consumers time this out instead.
    const auto safety_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    safety_status_publisher_ =
        node_->create_publisher<SafetyStatusMsg>("hardware_interface/safety_status", safety_qos);
    realtime_safety_status_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<SafetyStatusMsg>>(safety_status_publisher_);

    safety_command_echo_publisher_ = node_->create_publisher<SafetyCommandEchoMsg>(
        "hardware_interface/safety_command_echo", safety_qos);
    realtime_safety_command_echo_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<SafetyCommandEchoMsg>>(
            safety_command_echo_publisher_);

    diagnostic_updater_.setHardwareID("Rover System");

    RCLCPP_INFO(rclcpp::get_logger("SystemROSInterface"), "Node constructed successfully.");
}

SystemROSInterface::~SystemROSInterface()
{
    if (executor_) {
        executor_->cancel();

        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }

        executor_.reset();
    }

    realtime_driver_state_publisher_.reset();
    driver_state_publisher_.reset();

    realtime_safety_status_publisher_.reset();
    safety_status_publisher_.reset();
    realtime_safety_command_echo_publisher_.reset();
    safety_command_echo_publisher_.reset();

    service_wrappers_storage_.clear();
    node_.reset();
}

void SystemROSInterface::updateMsgErrorFlags(
    const DriverNames name,
    const DriverDataSnapshot & data)
{
    auto & driver_state = driver_state_msg_;
    auto & driver_state_named = getDriverStateByName(driver_state, name);

    driver_state.header.stamp = node_->get_clock()->now();

    driver_state_named.state.fault_flag = toFaultFlagMsg(data.getFaultFlag());
    driver_state_named.state.runtime_error =
        toRuntimeErrorMsg(data.getRuntimeError(MotorNames::DEFAULT));

    driver_state_named.state.motor_states_data_timed_out = data.isMotorStatesDataTimedOut();
    driver_state_named.state.driver_state_data_timed_out = data.isDriverStateDataTimedOut();
    driver_state_named.state.comm_error = data.isCommunicationError();
    // heartbeat_timeout intentionally left at its default (false): there is no heartbeat
    // protocol in this codebase to back it.
}

void SystemROSInterface::updateMsgDriversStates(
    const DriverNames name,
    const DriverStateReading & state)
{
    auto & driver_state = driver_state_msg_;
    auto & driver_state_named = getDriverStateByName(driver_state, name);

    driver_state_named.state.current = state.getDriverCurrent();
    driver_state_named.state.temperature = state.getTemperature();
}

void SystemROSInterface::updateMsgError(const bool error)
{
    driver_state_msg_.error = error;
}

void SystemROSInterface::publishRobotDriverState()
{
    realtime_driver_state_publisher_->try_publish(driver_state_msg_);
}

void SystemROSInterface::updateMsgGpioStates(
    const std::unordered_map<RoverControllerGpio, bool> & pin_state)
{
    for (const auto & [pin, pin_value] : pin_state) {
        updateSafetyMsgs(pin, pin_value);
    }
}

void SystemROSInterface::updateSafetyLinkState(const SafetyLinkHealth & health)
{
    const auto now = node_->get_clock()->now();

    safety_status_msg_.header.stamp = now;
    safety_command_echo_msg_.header.stamp = now;

    // io_sample_time is when the PLC was actually polled, which is what a staleness check should
    // be measuring. It is reconstructed from the poll age rather than stamped in the poll thread
    // because that thread runs on steady_clock and must not touch a ROS clock.
    const rclcpp::Time sample_time =
        (health.last_poll_age_ms == SafetyLinkHealth::kUnknownAgeMs)
            ? rclcpp::Time(0, 0, now.get_clock_type())
            : now - rclcpp::Duration(std::chrono::milliseconds(health.last_poll_age_ms));

    safety_status_msg_.io_sample_time = sample_time;
    safety_command_echo_msg_.io_sample_time = sample_time;

    // Both threads alive and a poll that has actually succeeded recently. Without this a consumer
    // could only infer link trouble from the message drying up, which is exactly the inference
    // the latched-topic mistake made unreliable.
    const bool poll_is_fresh = health.last_poll_age_ms != SafetyLinkHealth::kUnknownAgeMs &&
                               health.last_poll_age_ms <= kSafetyLinkStalePollAgeMs;

    safety_status_msg_.link_healthy =
        health.watchdog_running && health.poll_running && poll_is_fresh;
}

void SystemROSInterface::publishSafetyMsgs()
{
    realtime_safety_status_publisher_->try_publish(safety_status_msg_);
    realtime_safety_command_echo_publisher_->try_publish(safety_command_echo_msg_);
}

// Routes one pin to whichever of the two safety messages it belongs in. Returns false for pins
// that are not mapped into either - GPIO_1..7 and GPIO_14/15 are physically present on the
// controller but carry nothing this system uses.
bool SystemROSInterface::updateSafetyMsgs(const RoverControllerGpio pin, const bool pin_value)
{
    switch (pin) {
        // --- Plant state: what the hardware is doing. ---
        case RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN:
            safety_status_msg_.hw_e_stop_user_button = pin_value;
            break;
        case RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED:
            safety_status_msg_.motor_contactor_engaged = pin_value;
            break;
        case RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS:
            safety_status_msg_.latch_active = pin_value;
            break;

        // --- Command echoes: read-backs of coils we drive. Diagnostic only. ---
        case RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON:
            safety_command_echo_msg_.sw_e_stop_user_button = pin_value;
            break;
        case RoverControllerGpio::GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT:
            safety_command_echo_msg_.sw_e_stop_motor_driver_fault = pin_value;
            break;
        case RoverControllerGpio::GPIO_SW_E_STOP_LATCH_RESET:
            safety_command_echo_msg_.sw_e_stop_latch_reset = pin_value;
            break;
        case RoverControllerGpio::GPIO_CPU_WDG_HEARTBEAT:
            safety_command_echo_msg_.cpu_wdg_heartbeat = pin_value;
            break;

        default:
            return false;
    }

    return true;
}

rclcpp::CallbackGroup::SharedPtr SystemROSInterface::getOrCreateNodeCallbackGroup(
    const unsigned group_id, rclcpp::CallbackGroupType callback_group_type)
{
    if (group_id == 0) {

        if (callback_group_type == rclcpp::CallbackGroupType::Reentrant) {
            throw std::runtime_error(
                "Node callback group with id 0 (default group) cannot be of "
                "rclcpp::CallbackGroupType::Reentrant type.");
        }

        return nullptr;
    }

    const auto search = callback_groups_.find(group_id);

    if (search != callback_groups_.end()) {
        if (search->second->type() != callback_group_type) {
            throw std::runtime_error("Requested node callback group has incorrect type.");
        }

        return search->second;
    }

    auto callback_group = node_->create_callback_group(callback_group_type);
    callback_groups_[group_id] = callback_group;

    return callback_group;
}

DriverStateNamedMsg & SystemROSInterface::getDriverStateByName(
    RoverDriverStateMsg & robot_driver_state,
    const DriverNames name)
{
    const auto & name_str = cachedDriverName(name);
    auto & driver_states = robot_driver_state.driver_states;

    auto it = std::find_if(
        driver_states.begin(), driver_states.end(),
        [&name_str](const DriverStateNamedMsg & msg) {
            return msg.name == name_str;
    });

    if (it == driver_states.end()) {
        DriverStateNamedMsg driver_state_named;
        driver_state_named.name = name_str;
        driver_states.push_back(driver_state_named);
        it = driver_states.end() - 1;
    }

    return *it;
}

}  // namespace rover_hardware_interface
