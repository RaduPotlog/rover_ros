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

#ifndef ROVER_LED_LED_DRIVER_NODE_HPP_
#define ROVER_LED_LED_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rover_msgs/srv/set_led_brightness.hpp"

#include "rover_led/sk9822.hpp"
#include "rover_led/led_driver_parameters.hpp"

namespace rover_led
{

using ImageMsg = sensor_msgs::msg::Image;
using SetBoolSrv = std_srvs::srv::SetBool;
using SetLedBrightnessSrv = rover_msgs::srv::SetLedBrightness;

class LedDriverNode : public rclcpp::Node
{

public:

    LedDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    LedDriverNode(
        SK9822Interface::SharedPtr channel_1,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  
    void clearLeds();

    void toggleLedControlCallback(rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future);

    void frameCallback(
        const ImageMsg::UniquePtr & msg, 
        const SK9822Interface::SharedPtr & panel,
        const rclcpp::Time & last_time, 
        const std::string & panel_name);

    void panelThrottleWarnLog(const std::string panel_name, const std::string message);

    int channel_1_num_led_;
    double frame_timeout_;
    bool led_control_granted_;
    bool led_control_pending_;

    rclcpp::Time channel_1_ts_;

private:
  
    void onShutdown();

    void initializationTimerCallback();

    void toggleLedControl(const bool enable);

    void setBrightnessCallback(
        const SetLedBrightnessSrv::Request::SharedPtr & request,
        SetLedBrightnessSrv::Response::SharedPtr response);

    void diagnoseLeds(diagnostic_updater::DiagnosticStatusWrapper & status);

    static constexpr unsigned kMaxInitializationAttempts = 3;
    static constexpr unsigned kServiceResponseTimeout = 3;
    static constexpr unsigned kWaitForServiceTimeout = 3;

    unsigned initialization_attempt_;
    rclcpp::Time led_control_call_time_;

    SK9822Interface::SharedPtr channel_1_;

    std::shared_ptr<led_driver::ParamListener> param_listener_;
    led_driver::Params params_;

    rclcpp::TimerBase::SharedPtr initialization_timer_;

    rclcpp::Client<SetBoolSrv>::SharedPtr enable_led_control_client_;
    rclcpp::Service<SetLedBrightnessSrv>::SharedPtr set_brightness_server_;

    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    rclcpp::Subscription<ImageMsg>::SharedPtr channel_1_sub_;

    diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace rover_led

#endif  // ROVER_LED_LED_DRIVER_NODE_HPP_
