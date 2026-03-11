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

#ifndef ROVER_HARDWARE_INTERFACE_SYSTEM_ROS_INTERFACE_HPP_
#define ROVER_HARDWARE_INTERFACE_SYSTEM_ROS_INTERFACE_HPP_

#include <any>
#include <functional>
#include <map>
#include <memory>
#include <thread>
#include <unordered_map>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rover_msgs/msg/driver_state_named.hpp"
#include "rover_msgs/msg/rover_driver_state.hpp"
#include "rover_msgs/msg/gpio_state.hpp"

#include "rover_hardware_interface/rover_driver/driver.hpp"
#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_data_transformer.hpp"

#include "rover_hardware_interface/rover_modbus/modbus_types.hpp"
#include "rover_hardware_interface/rover_controller/rover_controller_types.hpp"

using namespace std::placeholders;

namespace rover_hardware_interface
{

// Standard messages
using BoolMsg = std_msgs::msg::Bool;
using SetBoolSrv = std_srvs::srv::SetBool;
using TriggerSrv = std_srvs::srv::Trigger;

// Rover messages
using RoverDriverStateMsg = rover_msgs::msg::RoverDriverState;
using DriverStateNamedMsg = rover_msgs::msg::DriverStateNamed;
using GpioStateMsg = rover_msgs::msg::GpioState;

template <typename SrvT, typename CallbackT>
class ROSServiceWrapper
{

public:

    using SrvSharedPtr = typename rclcpp::Service<SrvT>::SharedPtr;
    using SrvRequestConstPtr = typename SrvT::Request::ConstSharedPtr;
    using SrvResponsePtr = typename SrvT::Response::SharedPtr;

    ROSServiceWrapper(const CallbackT & callback) : callback_(callback) {}

    void registerService(
        const rclcpp::Node::SharedPtr node, const std::string & service_name,
        rclcpp::CallbackGroup::SharedPtr group = nullptr,
        const rclcpp::QoS & qos = rclcpp::ServicesQoS());

private:
    
    void callbackWrapper(SrvRequestConstPtr request, SrvResponsePtr response);
    void proccessCallback(SrvRequestConstPtr request);

    SrvSharedPtr service_;
    CallbackT callback_;
};

class SystemROSInterface
{

public:
  
    SystemROSInterface(const std::string & node_name,
                       const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
   
    ~SystemROSInterface();
    
    template <class SrvT, class CallbackT>
    inline void addService(
        const std::string & service_name, const CallbackT & callback, const unsigned group_id = 0,
        rclcpp::CallbackGroupType callback_group_type = rclcpp::CallbackGroupType::MutuallyExclusive,
        const rclcpp::QoS & qos = rclcpp::ServicesQoS())
    {
        rclcpp::CallbackGroup::SharedPtr callback_group = getOrCreateNodeCallbackGroup(
            group_id, callback_group_type);

        auto wrapper = std::make_shared<ROSServiceWrapper<SrvT, CallbackT>>(callback);
        wrapper->registerService(node_, service_name, callback_group, qos);
        service_wrappers_storage_.push_back(wrapper);
    }

    template <class T>
    inline void addDiagnosticTask(
    const std::string & name, T * task_owner,
    void (T::*task_fcn)(diagnostic_updater::DiagnosticStatusWrapper &))
    {
        diagnostic_updater_.add(name, task_owner, task_fcn);
    }

    inline void broadcastOnDiagnosticTasks(unsigned char level, const std::string & message)
    {
        diagnostic_updater_.broadcast(level, message);
    }

    void updateMsgErrorFlags(
        const DriverNames name, 
        const PhidgetDriverDataTransformer & data);

    void updateMsgDriversStates(
        const DriverNames name, 
        const PhidgetDriverStateTransformer & state);

    void publishRobotDriverState();

    void updateMsgGpioStates(
        const std::unordered_map<RoverControllerGpio, bool> & pin_state);

    void publishGpioStateMsg();

protected:

    bool updateGpioStateMsg(const RoverControllerGpio pin, const bool pin_value);

    rclcpp::CallbackGroup::SharedPtr getOrCreateNodeCallbackGroup(const unsigned group_id, rclcpp::CallbackGroupType callback_group_type);

    DriverStateNamedMsg & getDriverStateByName(
        RoverDriverStateMsg & robot_driver_state, 
        const DriverNames name);
    
    rclcpp::Node::SharedPtr node_;
    std::unordered_map<unsigned, rclcpp::CallbackGroup::SharedPtr> callback_groups_;
    rclcpp::executors::MultiThreadedExecutor::UniquePtr executor_;
    std::thread executor_thread_;

    rclcpp::Publisher<RoverDriverStateMsg>::SharedPtr driver_state_publisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<RoverDriverStateMsg>>realtime_driver_state_publisher_;

    rclcpp::Publisher<GpioStateMsg>::SharedPtr gpio_state_publisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<GpioStateMsg>> realtime_gpio_state_publisher_;

    diagnostic_updater::Updater diagnostic_updater_;

    std::vector<std::any> service_wrappers_storage_;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_SYSTEM_ROS_INTERFACE_HPP_
