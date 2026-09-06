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

#include <algorithm>
#include <string>
#include <chrono>
#include <iostream>

#include "rover_crfs_teleop/rover_crfs_teleop_controller.hpp"

namespace rover_crfs_telop
{

using namespace std::string_literals;
using namespace std::chrono_literals;

RoverCrfsTeleopController::RoverCrfsTeleopController(const std::string & node_name)
: Node(node_name)
{
    RCLCPP_INFO(this->get_logger(), "RoverCrfsTeleopController node constructed");

    declareAndReadParameters();

    auto topic_callback = [this](crsf_receiver_msg::msg::CRSFChannels16::UniquePtr msg) -> void {

        channels_values_[0] = msg->ch1;
        channels_values_[1] = msg->ch2;
        channels_values_[2] = msg->ch3;
        channels_values_[3] = msg->ch4;
        channels_values_[4] = msg->ch5;
        channels_values_[5] = msg->ch6;
        channels_values_[6] = msg->ch7;
        channels_values_[7] = msg->ch8;
        channels_values_[8] = msg->ch9;
        channels_values_[9] = msg->ch10;
        channels_values_[10] = msg->ch11;
        channels_values_[11] = msg->ch12;
        channels_values_[12] = msg->ch13;
        channels_values_[13] = msg->ch14;
        channels_values_[14] = msg->ch15;
        channels_values_[15] = msg->ch16;

        channels_received_ = true;
    };
    
    auto rc_link_topic_callback = [this](crsf_receiver_msg::msg::CRSFLinkInfo::UniquePtr msg) -> void {
        
        this->uplink_rssi_ant1_ = msg->uplink_rssi_ant1.data;
        this->uplink_rssi_ant2_ = msg->uplink_rssi_ant2.data;
    };
    
    chanel_subscriber_ = this->create_subscription<crsf_receiver_msg::msg::CRSFChannels16>(
        "rc/channels", rclcpp::QoS(1).best_effort().durability_volatile(), topic_callback);

    rc_link_subscriber_ = this->create_subscription<crsf_receiver_msg::msg::CRSFLinkInfo>(
        "rc/link", rclcpp::QoS(1).best_effort().durability_volatile(), rc_link_topic_callback);

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "teleop_elrs_cmd_vel_stamped", 10
    );

    e_stop_set_set_ = this->create_client<std_srvs::srv::Trigger>("hardware_interface/sw_user_e_stop_set");
    e_stop_set_reset_ = this->create_client<std_srvs::srv::Trigger>("hardware_interface/sw_user_e_stop_reset");
    sw_e_stop_latch_reset_ = this->create_client<std_srvs::srv::Trigger>("hardware_interface/sw_e_stop_latch_reset");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), 
        std::bind(&RoverCrfsTeleopController::main_timer_callback, this)
    );

    rssi_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(400), 
        std::bind(&RoverCrfsTeleopController::rssi_timer_callback, this)
    );
}

void RoverCrfsTeleopController::restart_rssi__timer()
{
    // Call reset to restart a cancelled timer or re-trigger a running one
    if (rssi_timer_->is_canceled()) {
        rssi_timer_->reset();
    } else {
        RCLCPP_INFO(this->get_logger(), "Time reached: set uplink_rssi_ant = 0!");
    }
}

void RoverCrfsTeleopController::rssi_timer_callback()
{
    this->uplink_rssi_ant1_ = 0;
    this->uplink_rssi_ant2_ = 0;
}

void RoverCrfsTeleopController::declareAndReadParameters()
{
    // Every parameter is declared, per .claude/rules/ros2_general.md - a silent get_parameter()
    // on an undeclared name is a bug. Defaults are the raw CRSF endpoints from
    // crsf_receiver/include/crsf_protocol.h, which is what rc/channels actually carries.
    const int channel_in_min = this->declare_parameter<int>("channel_in_min", kDefaultCrsfChannelMin);
    const int channel_in_mid = this->declare_parameter<int>("channel_in_mid", kDefaultCrsfChannelMid);
    const int channel_in_max = this->declare_parameter<int>("channel_in_max", kDefaultCrsfChannelMax);
    const int channel_deadband =
        this->declare_parameter<int>("channel_deadband", kDefaultChannelDeadband);

    linear_x_channel_ = this->declare_parameter<int>("linear_x_channel", 3);
    angular_z_channel_ = this->declare_parameter<int>("angular_z_channel", 1);
    e_stop_channel_ = this->declare_parameter<int>("e_stop_channel", 5);
    e_stop_latch_reset_channel_ = this->declare_parameter<int>("e_stop_latch_reset_channel", 4);
    channel_switch_threshold_ = this->declare_parameter<int>("channel_switch_threshold", 500);

    linear_x_mapping_.in_min = channel_in_min;
    linear_x_mapping_.in_mid = channel_in_mid;
    linear_x_mapping_.in_max = channel_in_max;
    linear_x_mapping_.deadband_counts = channel_deadband;
    linear_x_mapping_.out_min = this->declare_parameter<double>("linear_x_out_min", -2.0);
    linear_x_mapping_.out_max = this->declare_parameter<double>("linear_x_out_max", 2.0);
    linear_x_mapping_.invert = this->declare_parameter<bool>("linear_x_invert", false);

    angular_z_mapping_.in_min = channel_in_min;
    angular_z_mapping_.in_mid = channel_in_mid;
    angular_z_mapping_.in_max = channel_in_max;
    angular_z_mapping_.deadband_counts = channel_deadband;
    angular_z_mapping_.out_min = this->declare_parameter<double>("angular_z_out_min", -5.0);
    angular_z_mapping_.out_max = this->declare_parameter<double>("angular_z_out_max", 5.0);
    // The previous implementation inverted this axis by swapping in_min/in_max; the mapping is
    // defined about the midpoint now, so the inversion is explicit.
    angular_z_mapping_.invert = this->declare_parameter<bool>("angular_z_invert", true);

    channels_values_.fill(channel_in_mid);

    RCLCPP_INFO(
        this->get_logger(),
        "RC channel range [%d, %d] with midpoint %d and deadband %d counts.",
        channel_in_min, channel_in_max, channel_in_mid, channel_deadband);
}

int RoverCrfsTeleopController::channelValue(const int channel_number) const
{
    if (channel_number < 1 || static_cast<std::size_t>(channel_number) > kChannelCount) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Configured channel number %d is outside 1-%zu; treating it as 0.",
            channel_number, kChannelCount);
        return 0;
    }

    return channels_values_[static_cast<std::size_t>(channel_number - 1)];
}

void RoverCrfsTeleopController::callTriggerService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & description)
{
    if (!client->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Service for '%s' is not available; request dropped.", description.c_str());
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // The response used to be discarded, so a refused request (e.g. the hardware interface
    // rejecting an E-Stop reset because the rover is still being commanded to move) was
    // invisible to the operator - and, because the request only re-fires on a channel edge,
    // there was no retry either. Report it.
    client->async_send_request(
        request,
        [this, description](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            const auto response = future.get();

            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "%s succeeded.", description.c_str());
            } else {
                RCLCPP_WARN(
                    this->get_logger(), "%s was refused: %s", description.c_str(),
                    response->message.c_str());
            }
        });
}

void RoverCrfsTeleopController::handleEStopChannel()
{
    const int current_value = channelValue(e_stop_channel_);

    // Latch the switch's resting position on the first frames instead of treating it as an edge,
    // which would fire a spurious set-or-reset at startup. Mirrors the latch-reset channel below.
    if (!e_stop_init) {
        e_stop_old_value = current_value;

        if (e_stop_init_counter > 0) {
            e_stop_init_counter--;
        } else {
            e_stop_init = true;
        }

        return;
    }

    if (current_value == e_stop_old_value) {
        return;
    }

    e_stop_old_value = current_value;

    if (current_value < channel_switch_threshold_) {
        callTriggerService(e_stop_set_set_, "SW User E-Stop set");
    } else {
        callTriggerService(e_stop_set_reset_, "SW User E-Stop reset");
    }
}

void RoverCrfsTeleopController::handleEStopLatchResetChannel()
{
    const int current_value = channelValue(e_stop_latch_reset_channel_);

    if (!e_stop_latch_reset_init) {
        e_stop_latch_reset_old_value = current_value;

        if (e_stop_latch_reset_init_counter > 0) {
            e_stop_latch_reset_init_counter--;
        } else {
            e_stop_latch_reset_init = true;
        }

        return;
    }

    if (current_value == e_stop_latch_reset_old_value) {
        return;
    }

    e_stop_latch_reset_old_value = current_value;

    if (current_value < channel_switch_threshold_) {
        callTriggerService(sw_e_stop_latch_reset_, "SW E-Stop latch reset");
    }
}

void RoverCrfsTeleopController::main_timer_callback()
{
    if (!channels_received_) {
        return;
    }

    if ((this->uplink_rssi_ant1_ <= 5) && (this->uplink_rssi_ant2_ < 5)) return;

    const double twist_linear_x = mapAxis(channelValue(linear_x_channel_), linear_x_mapping_);
    const double twist_angular_z = mapAxis(channelValue(angular_z_channel_), angular_z_mapping_);

    auto msg = geometry_msgs::msg::TwistStamped();

    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x  = twist_linear_x;
    msg.twist.linear.y  = 0.0;
    msg.twist.linear.z  = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = twist_angular_z;

    // Publish just once the command in case of zero to not block mux. This only works because a
    // centred stick now maps to exactly 0.0 - see domain/stick_mapping.hpp.
    if (!twist_pub_just_once_) {
        cmd_vel_publisher_->publish(msg);
    }

    if ((msg.twist.linear.x == 0.0) && (msg.twist.angular.z == 0.0)) {
        twist_pub_just_once_ = true;
    }
    else {
        twist_pub_just_once_ = false;
    }

    handleEStopChannel();
    handleEStopLatchResetChannel();
}

} // rover_crfs_telop