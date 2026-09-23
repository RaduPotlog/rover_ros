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
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rover_msgs/msg/driver_state_named.hpp"
#include "rover_msgs/msg/rover_driver_state.hpp"
#include "rover_hardware_interface/domain/safety_link_health.hpp"
#include "rover_msgs/msg/aux_io_state.hpp"
#include "rover_msgs/msg/safety_command_echo.hpp"
#include "rover_msgs/msg/safety_status.hpp"

#include "rover_hardware_interface/domain/driver.hpp"
#include "rover_hardware_interface/domain/driver_data_snapshot.hpp"

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_types.hpp"

namespace rover_hardware_interface
{

// Standard messages
using BoolMsg = std_msgs::msg::Bool;
using TriggerSrv = std_srvs::srv::Trigger;

// Rover messages
using RoverDriverStateMsg = rover_msgs::msg::RoverDriverState;
using DriverStateNamedMsg = rover_msgs::msg::DriverStateNamed;
using SafetyStatusMsg = rover_msgs::msg::SafetyStatus;
using SafetyCommandEchoMsg = rover_msgs::msg::SafetyCommandEcho;
using AuxIoStateMsg = rover_msgs::msg::AuxIoState;
using SetBoolSrv = std_srvs::srv::SetBool;

template <typename SrvT, typename CallbackT>
class ROSServiceWrapper
{

public:

    using SrvSharedPtr = typename rclcpp::Service<SrvT>::SharedPtr;
    using SrvRequestConstPtr = typename SrvT::Request::ConstSharedPtr;
    using SrvResponsePtr = typename SrvT::Response::SharedPtr;

    explicit ROSServiceWrapper(const CallbackT & callback) : callback_(callback) {}

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
        const DriverDataSnapshot & data);

    void updateMsgDriversStates(
        const DriverNames name,
        const DriverStateReading & state);

    void updateMsgError(const bool error);

    void publishRobotDriverState();

    // Fans the raw pin map out across the two safety messages. Which pin lands in which is the
    // whole point of the split: plant readings go to SafetyStatus, echoes of coils we drive go to
    // SafetyCommandEcho. See updateSafetyMsgs(). The general-purpose aux pins go to AuxIoState.
    void updateMsgGpioStates(
        const std::unordered_map<RoverControllerGpio, bool> & pin_state);

    // Fills in the parts that do not come from a pin: header stamps, io_sample_time (derived from
    // how long ago the PLC was actually polled) and link_healthy.
    void updateSafetyLinkState(const SafetyLinkHealth & health);

    void publishSafetyMsgs();

protected:

    bool updateSafetyMsgs(const RoverControllerGpio pin, const bool pin_value);

    rclcpp::CallbackGroup::SharedPtr getOrCreateNodeCallbackGroup(const unsigned group_id, rclcpp::CallbackGroupType callback_group_type);

    DriverStateNamedMsg & getDriverStateByName(
        RoverDriverStateMsg & robot_driver_state,
        const DriverNames name);

    rclcpp::Node::SharedPtr node_;
    std::unordered_map<unsigned, rclcpp::CallbackGroup::SharedPtr> callback_groups_;
    rclcpp::executors::MultiThreadedExecutor::UniquePtr executor_;
    std::thread executor_thread_;

    // Staging messages owned by the update thread; try_publish copies them under its lock.
    RoverDriverStateMsg driver_state_msg_;
    SafetyStatusMsg safety_status_msg_;
    SafetyCommandEchoMsg safety_command_echo_msg_;
    AuxIoStateMsg aux_io_state_msg_;

    rclcpp::Publisher<RoverDriverStateMsg>::SharedPtr driver_state_publisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<RoverDriverStateMsg>> realtime_driver_state_publisher_;

    rclcpp::Publisher<SafetyStatusMsg>::SharedPtr safety_status_publisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<SafetyStatusMsg>> realtime_safety_status_publisher_;

    rclcpp::Publisher<SafetyCommandEchoMsg>::SharedPtr safety_command_echo_publisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<SafetyCommandEchoMsg>>
        realtime_safety_command_echo_publisher_;

    rclcpp::Publisher<AuxIoStateMsg>::SharedPtr aux_io_state_publisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<AuxIoStateMsg>> realtime_aux_io_state_publisher_;

    diagnostic_updater::Updater diagnostic_updater_;

    std::vector<std::any> service_wrappers_storage_;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_SYSTEM_ROS_INTERFACE_HPP_
