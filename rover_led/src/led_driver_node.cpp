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

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rover_led/led_driver_node.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rover_msgs/srv/set_led_brightness.hpp"

#include "rover_led/sk9822.hpp"
#include "rover_led/led_controller_parameters.hpp"

namespace rover_led
{

using std::placeholders::_1;
using std::placeholders::_2;

LedDriverNode::LedDriverNode(const rclcpp::NodeOptions & options)
: Node("led_driver", options)
, led_control_granted_(true)
, led_control_pending_(false)
, initialization_attempt_(0)
, channel_1_(std::make_shared<SK9822>())
, channel_2_(std::make_shared<SK9822>())
, diagnostic_updater_(this)
{
    RCLCPP_INFO(this->get_logger(), "Constructing node.");

    rclcpp::on_shutdown(std::bind(&LedDriverNode::onShutdown, this));

    this->param_listener_ =
    std::make_shared<led_driver::ParamListener>(this->get_node_parameters_interface());
    this->params_ = this->param_listener_->get_params();

    frame_timeout_ = this->params_.frame_timeout;
    channel_1_num_led_ = this->params_.channel_1_num_led;
    channel_2_num_led_ = this->params_.channel_2_num_led;

    const float global_brightness = this->params_.global_brightness;
    channel_1_->setGlobalBrightness(global_brightness);
    channel_2_->setGlobalBrightness(global_brightness);

    client_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    enable_led_control_client_ = this->create_client<SetBoolSrv>(
        "hardware/led_control_enable", rclcpp::ServicesQoS(), client_callback_group_);

    set_brightness_server_ = this->create_service<SetLedBrightnessSrv>(
        "led/set_brightness", std::bind(&LedDriverNode::setBrightnessCallback, this, _1, _2));

    initialization_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&LedDriverNode::initializationTimerCallback, this));

    channel_1_ts_ = this->get_clock()->now();
    channel_1_sub_ = this->create_subscription<ImageMsg>(
        "led/channel_1_frame", 5, [&](const ImageMsg::UniquePtr & msg) {
            frameCallback(msg, channel_1_, channel_1_ts_, "channel_1");
            channel_1_ts_ = msg->header.stamp;
    });

    channel_2_ts_ = this->get_clock()->now();
    channel_2_sub_ = this->create_subscription<ImageMsg>(
        "led/channel_2_frame", 5, [&](const ImageMsg::UniquePtr & msg) {
            frameCallback(msg, channel_2_, channel_2_ts_, "channel_2");
            channel_2_ts_ = msg->header.stamp;
    });

    channel_1_pub_ = this->create_publisher<UdpPacketMsg>("udp_write/led_channel_1", 5);
    channel_2_pub_ = this->create_publisher<UdpPacketMsg>("udp_write/led_channel_2", 5);

    diagnostic_updater_.setHardwareID("Bumper Led");
    diagnostic_updater_.add("Led driver status", this, &LedDriverNode::diagnoseLeds);

    led_control_pending_ = true;
    led_control_call_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void LedDriverNode::onShutdown()
{
    clearLeds();

    if (led_control_granted_) {
        toggleLedControl(false);
    }
}

void LedDriverNode::initializationTimerCallback()
{
    if (led_control_granted_) {
        initialization_timer_->cancel();
        return;
    }

    if (led_control_pending_) {
        if (this->now() - led_control_call_time_ <= rclcpp::Duration(std::chrono::seconds(kServiceResponseTimeout))) {
            return;
        }

        RCLCPP_WARN(this->get_logger(), "LED control service response timeout.");
    
        led_control_pending_ = false;
    }

    if (initialization_attempt_ >= kMaxInitializationAttempts) {
        throw std::runtime_error("Failed to initialize LED driver.");
    }

    toggleLedControl(true);

    initialization_attempt_++;
}

void LedDriverNode::clearLeds()
{
    const auto buffer_channel_1 = channel_1_->setPanel(std::vector<std::uint8_t>(channel_1_num_led_ * 4, 0));
    const auto buffer_channel_2 = channel_2_->setPanel(std::vector<std::uint8_t>(channel_2_num_led_ * 4, 0));

    channel_1_msg_.address = channel_1_ip_address_;
    channel_1_msg_.src_port = channel_1_src_port_;
    channel_1_msg_.data = buffer_channel_1;
    
    channel_2_msg_.address = channel_2_ip_address_;
    channel_2_msg_.src_port = channel_2_src_port_;
    channel_2_msg_.data = buffer_channel_2;
    
    auto now = this->now();
    
    channel_1_msg_.header.stamp.sec = now.seconds();
    channel_1_msg_.header.stamp.nanosec = now.nanoseconds();
    channel_1_msg_.header.frame_id = "";
    
    channel_2_msg_.header.stamp.sec = now.seconds();
    channel_2_msg_.header.stamp.nanosec = now.nanoseconds();
    channel_2_msg_.header.frame_id = "";
    
    channel_1_pub_->publish(channel_1_msg_);
    channel_2_pub_->publish(channel_2_msg_);
}

void LedDriverNode::toggleLedControl(const bool enable)
{
    RCLCPP_DEBUG(this->get_logger(), "Calling service to toggle LED control to '%s'.", enable ? "true" : "false");

    auto request = std::make_shared<SetBoolSrv::Request>();
    request->data = enable;

    if (!enable_led_control_client_->wait_for_service(std::chrono::seconds(kWaitForServiceTimeout))) {
        RCLCPP_WARN_STREAM(
            this->get_logger(), "Timeout occurred while waiting for service '" 
                << enable_led_control_client_->get_service_name() << "'!");
        return;
    }

    enable_led_control_client_->async_send_request(
    request, std::bind(&LedDriverNode::toggleLedControlCallback, this, std::placeholders::_1));

    led_control_pending_ = true;
    led_control_call_time_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "Sent request toggling LED control to '%s'.", enable ? "true" : "false");
}

void LedDriverNode::toggleLedControlCallback(rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future)
{
    RCLCPP_DEBUG(this->get_logger(), "Received response after toggling LED control.");

    const auto result = future.get();

    const auto request = result.first;
    const auto response = result.second;

    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to toggle LED control.");
        led_control_pending_ = false;
        return;
    }

    if (request->data == true) {
        led_control_granted_ = true;
        clearLeds();
        RCLCPP_INFO(this->get_logger(), "LED control granted.");
    } else {
        led_control_granted_ = false;
        RCLCPP_INFO(this->get_logger(), "LED control revoked.");
    }

    led_control_pending_ = false;
}

void LedDriverNode::frameCallback(
    const ImageMsg::UniquePtr & msg, 
    const SK9822Interface::SharedPtr & panel,
    const rclcpp::Time & last_time, 
    const std::string & panel_name)
{
    if (!led_control_granted_) {
        panelThrottleWarnLog(panel_name, "Waiting for LED control to be granted. Ignoring frame for " + panel_name + "!");
        return;
    }

    std::string message;
  
    if ((this->get_clock()->now() - rclcpp::Time(msg->header.stamp)) > rclcpp::Duration::from_seconds(frame_timeout_)) {
        message = "Timeout exceeded, ignoring frame";
    } else if (rclcpp::Time(msg->header.stamp) < last_time) {
        message = "Dropping message from past";
    } else if (msg->encoding != sensor_msgs::image_encodings::RGBA8) {
        message = "Incorrect image encoding ('" + msg->encoding + "')";
    } else if (msg->height != 1) {
        message = "Incorrect image height " + std::to_string(msg->height);
    } else if (msg->width != static_cast<std::uint32_t>(panel_name == "channel_1" ? channel_1_num_led_ : channel_2_num_led_)) {
        message = "Incorrect image width " + std::to_string(msg->width);
    }

    if (!message.empty()) {
        auto warn_msg = message + " on " + panel_name + "!";
        panelThrottleWarnLog(panel_name, warn_msg);
        diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, warn_msg);
        return;
    }
    
    const auto buffer = panel->setPanel(msg->data);
    auto now = this->now();
        
    udp_msgs::msg::UdpPacket udp_msg;
    udp_msg.header.stamp.sec = now.seconds();
    udp_msg.header.stamp.nanosec = now.nanoseconds();
    udp_msg.header.frame_id = "";
    
    for (size_t i = 0; i < buffer.size(); i += 4) {                
        udp_msg.data.push_back(buffer[i + 1]);
        udp_msg.data.push_back(buffer[i + 2]);
        udp_msg.data.push_back(buffer[i + 3]);
        udp_msg.data.push_back(buffer[i + 0]);
    }
    
    message.clear();
    
    if (panel == channel_1_) {
        udp_msg.address = channel_1_ip_address_;
        udp_msg.src_port = channel_1_src_port_;
        channel_1_pub_->publish(udp_msg);
    } else if (panel == channel_2_){
        udp_msg.address = channel_2_ip_address_;
        udp_msg.src_port = channel_2_src_port_;
        channel_2_pub_->publish(udp_msg);
    } else {
        message = "Incorrect pannel";
    }

    if (!message.empty()) {
        auto warn_msg = message + " on " + panel_name + "!";
        panelThrottleWarnLog(panel_name, warn_msg);
        diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, warn_msg);
    }
}

void LedDriverNode::setBrightnessCallback(
    const SetLedBrightnessSrv::Request::SharedPtr & req, 
    SetLedBrightnessSrv::Response::SharedPtr res)
{
    const float brightness = req->data;

    try {
        channel_1_->setGlobalBrightness(brightness);
        channel_2_->setGlobalBrightness(brightness);
    } catch (const std::out_of_range & e) {
        res->success = false;
        res->message = "Failed to set brightness: " + std::string(e.what());
        return;
    }

    auto str_bright = std::to_string(brightness);
    str_bright = str_bright.substr(0, str_bright.find(".") + 3);
    res->success = true;
    res->message = "Changed brightness to " + str_bright;
}

void LedDriverNode::panelThrottleWarnLog(
    const std::string panel_name, 
    const std::string message)
{
    if (panel_name == "channel_1") {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, message);
    } else if (panel_name == "channel_2") {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, message);
    }
}

void LedDriverNode::diagnoseLeds(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::ERROR};
    std::string message{"Driver is not functional!"};
    std::string led_control_status{"NOT_GRANTED"};

    if (led_control_granted_) {
        error_level = diagnostic_updater::DiagnosticStatusWrapper::OK;
        message = "Driver is fully functional.";
        led_control_status = "GRANTED";
    } else if (led_control_pending_) {
        error_level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
        message = "Driver is not yet functional!";
        led_control_status = "PENDING";
    }

    status.add("LED control status", led_control_status);
    status.summary(error_level, message);
}

}  // namespace rover_led

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rover_led::LedDriverNode)