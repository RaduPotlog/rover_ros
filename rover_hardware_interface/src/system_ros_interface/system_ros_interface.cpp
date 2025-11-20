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

#include <memory>
#include <string>
#include <thread>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace rover_hardware_interface
{

template class ROSServiceWrapper<std_srvs::srv::SetBool, std::function<void(bool)>>;
template class ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>;

template <typename SrvT, typename CallbackT>
void ROSServiceWrapper<SrvT, CallbackT>::registerService(
    const rclcpp::Node::SharedPtr node, const std::string & service_name,
    rclcpp::CallbackGroup::SharedPtr group, const rclcpp::QoS & qos)
{
    service_ = node->create_service<SrvT>(
        service_name, 
        std::bind(&ROSServiceWrapper<SrvT, CallbackT>::callbackWrapper, this, _1, _2),
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
            rclcpp::get_logger("RoverSystem"),
            "An exception occurred while handling the request: " << err.what());
    }
}

template <>
void ROSServiceWrapper<std_srvs::srv::SetBool, std::function<void(bool)>>::proccessCallback(SrvRequestConstPtr request)
{
    callback_(request->data);
}

template <>
void ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>::proccessCallback(SrvRequestConstPtr /* request */)
{
    callback_();
}

SystemROSInterface::SystemROSInterface(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: node_(rclcpp::Node::make_shared(node_name, node_options)), diagnostic_updater_(node_)
{
    RCLCPP_INFO(rclcpp::get_logger("RoverSystem"), "Constructing node.");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);

    executor_thread_ = std::thread([this]() { 
        executor_->spin(); }
    );

    driver_state_publisher_ = node_->create_publisher<RoverDriverStateMsg>("hardware_interface/rover_driver_state", 5);
    realtime_driver_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<RoverDriverStateMsg>>(driver_state_publisher_);

    gpio_state_publisher_ = node_->create_publisher<GpioStateMsg>("hardware_interface/gpio_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    realtime_gpio_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<GpioStateMsg>>(gpio_state_publisher_);
    
    diagnostic_updater_.setHardwareID("Rover System");
    
    RCLCPP_INFO(rclcpp::get_logger("RoverSystem"), "Node constructed successfully.");
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

    service_wrappers_storage_.clear();
    node_.reset();
}

void SystemROSInterface::updateMsgErrorFlags(
    const DriverNames name, 
    const PhidgetDriverDataTransformer & data)
{
    auto & driver_state = realtime_driver_state_publisher_->msg_;
    auto & driver_state_named = getDriverStateByName(driver_state, name);

    driver_state.header.stamp = node_->get_clock()->now();

    driver_state_named.state.fault_flag = 
        data.getFaultFlag().getMessage();
    driver_state_named.state.runtime_error =
        data.getRuntimeError(PhidgetDriver::motorChannelDefault).getMessage();
}

void SystemROSInterface::updateMsgDriversStates(
    const DriverNames name, 
    const PhidgetDriverStateTransformer & state)
{
    auto & driver_state = realtime_driver_state_publisher_->msg_;
    auto & driver_state_named = getDriverStateByName(driver_state, name);

    driver_state_named.state.current = state.getDriverCurrent();
    driver_state_named.state.temperature = state.getTemperature();
}

void SystemROSInterface::publishRobotDriverState()
{
    if (realtime_driver_state_publisher_->trylock()) {
        realtime_driver_state_publisher_->unlockAndPublish();
    }
}

void SystemROSInterface::updateMsgGpioStates(
    const std::unordered_map<RoverControllerGpio, bool> & pin_state)
{
    for (const auto & [pin, pin_value] : pin_state) {
        updateGpioStateMsg(pin, pin_value);
    }
}

void SystemROSInterface::publishGpioStateMsg()
{
    if (realtime_gpio_state_publisher_->trylock()) {
        realtime_gpio_state_publisher_->unlockAndPublish();
    }
}

bool SystemROSInterface::updateGpioStateMsg(const RoverControllerGpio pin, const bool pin_value)
{
    auto & pin_state_msg = realtime_gpio_state_publisher_->msg_;

    switch (pin) {
        case RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN:
            pin_state_msg.gpio_pin_hw_e_stop_user_button = pin_value;
            break;
        case RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED:
            pin_state_msg.gpio_pin_motor_contactor_engaged = pin_value;
            break;
        case RoverControllerGpio::GPIO_SW_E_STOP_CPU_WDG_TRIGGER:
            pin_state_msg.gpio_pin_sw_e_stop_cpu_wdg_trigger = pin_value;
            break;
        case RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON:
            pin_state_msg.gpio_pin_sw_e_stop_user_button = pin_value;
            break;
        case RoverControllerGpio::GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT:
            pin_state_msg.gpio_pin_sw_e_stop_motor_driver_fault = pin_value;
            break;
        case RoverControllerGpio::GPIO_SW_E_STOP_LATCH_RESET:
            pin_state_msg.gpio_pin_sw_e_stop_latch_reset = pin_value;
            break;
        case RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS:
            pin_state_msg.gpio_pin_sw_e_stop_latch_status = pin_value;
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
    const auto name_str = driverNamesToString(name);
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
