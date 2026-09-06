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

#ifndef ROVER_CRFS_TELEOP_ROVER_CRFS_TELEOP_CONTROLLER_HPP_
#define ROVER_CRFS_TELEOP_ROVER_CRFS_TELEOP_CONTROLLER_HPP_

#include <stdio.h>
#include <array>
#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>
#include "rclcpp/qos.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "crsf_receiver_msg/msg/crsf_channels16.hpp"
#include "crsf_receiver_msg/msg/crsf_link_info.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "rover_crfs_teleop/domain/stick_mapping.hpp"

namespace rover_crfs_telop
{

class RoverCrfsTeleopController : public rclcpp::Node 
{

public:

    static constexpr std::size_t kChannelCount = 16;

    RoverCrfsTeleopController(const std::string & node_name);

private:

    void main_timer_callback();

    void rssi_timer_callback();

    void restart_rssi__timer();

    void declareAndReadParameters();

    // Returns the raw value of the configured channel number (1-16) out of the last received
    // frame, or 0 for an out-of-range channel number.
    int channelValue(const int channel_number) const;

    // Fires `client` if - and only if - it is ready. Deliberately non-blocking: this runs inside
    // the 20 ms main timer, where the previous `while (!client->wait_for_service(1s))` loop
    // stalled the whole executor (teleop included) for seconds at a time whenever a service was
    // briefly unavailable.
    void callTriggerService(
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
        const std::string & description);

    void handleEStopChannel();
    void handleEStopLatchResetChannel();

    rclcpp::Subscription<crsf_receiver_msg::msg::CRSFChannels16>::SharedPtr chanel_subscriber_;
    rclcpp::Subscription<crsf_receiver_msg::msg::CRSFLinkInfo>::SharedPtr rc_link_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_set_reset_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_set_set_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sw_e_stop_latch_reset_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr rssi_timer_;

    // Last received frame, indexed 0-based (channel N lives at index N-1). Replaces the previous
    // 11-bit bitfield struct: the channels the node reads are ROS-configurable now, and
    // rc/channels carries plain int32 values anyway.
    std::array<int, kChannelCount> channels_values_{};
    // Nothing is published until a real frame has arrived: the default-constructed array above
    // would otherwise clamp to in_min, i.e. full negative deflection on both axes.
    bool channels_received_{false};

    int e_stop_old_value{0};
    // Mirrors e_stop_latch_reset_init below: without it the very first frame received always
    // looks like a channel edge against the initial 0, and the node fires a spurious E-Stop
    // set-or-reset at startup purely from the switch's resting position.
    bool e_stop_init{false};
    unsigned int e_stop_init_counter{100};


    int e_stop_latch_reset_old_value{0};
    bool e_stop_latch_reset_init{false};
    unsigned int e_stop_latch_reset_init_counter{100};

    // Stick -> twist mapping, built from ROS parameters in declareAndReadParameters(). See
    // domain/stick_mapping.hpp for why this is defined about the channel midpoint.
    AxisMapping linear_x_mapping_;
    AxisMapping angular_z_mapping_;

    int linear_x_channel_{3};
    int angular_z_channel_{1};
    int e_stop_channel_{5};
    int e_stop_latch_reset_channel_{4};
    int channel_switch_threshold_{500};

    uint8_t uplink_rssi_ant1_{0};
    uint8_t uplink_rssi_ant2_{0};

    bool twist_pub_just_once_{false};
};

}  // namespace rover_crfs_telop

#endif  // ROVER_CRFS_TELEOP_ROVER_CRFS_TELEOP_CONTROLLER_HPP_