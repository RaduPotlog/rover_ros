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

#ifndef ROVER_DIAG_MANAGER_SYSTEM_DIAG_SYSTEM_DIAG_NODE_HPP_
#define ROVER_DIAG_MANAGER_SYSTEM_DIAG_SYSTEM_DIAG_NODE_HPP_

#include <stdio.h>
#include <chrono>
#include <functional>
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

namespace rover_crfs_telop
{

class RoverCrfsTeleopController : public rclcpp::Node 
{

public:

    struct RoverCrfsTeleopControllerChannels
    {
        unsigned ch1 : 11;
        unsigned ch2 : 11;
        unsigned ch3 : 11;
        unsigned ch4 : 11;
        unsigned ch5 : 11;
        unsigned ch6 : 11;
        unsigned ch7 : 11;
        unsigned ch8 : 11;
        unsigned ch9 : 11;
        unsigned ch10 : 11;
        unsigned ch11 : 11;
        unsigned ch12 : 11;
        unsigned ch13 : 11;
        unsigned ch14 : 11;
        unsigned ch15 : 11;
        unsigned ch16 : 11;
    } PACKED;

    RoverCrfsTeleopController(const std::string & node_name);

private:

    void main_timer_callback();

    void rssi_timer_callback();

    void restart_rssi__timer();

    rclcpp::Subscription<crsf_receiver_msg::msg::CRSFChannels16>::SharedPtr chanel_subscriber_;
    rclcpp::Subscription<crsf_receiver_msg::msg::CRSFLinkInfo>::SharedPtr rc_link_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_set_reset_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr e_stop_set_set_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sw_e_stop_latch_reset_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr rssi_timer_;

    RoverCrfsTeleopControllerChannels channels_values_;

    unsigned int e_stop_old_value{0};
    unsigned int e_stop_latch_reset_old_value{0};
    bool e_stop_latch_reset_init{false};
    unsigned int e_stop_latch_reset_init_counter{100};

    float twist_linear_x_in_min{0.0f};
    float twist_linear_x_in_max{2000.0f};
    
    float twist_linear_x_out_min{-2.0f};
    float twist_linear_x_out_max{2.0f};
    
    float twist_angular_z_in_min{2000.0f};
    float twist_angular_z_in_max{0.0f};

    float twist_angular_z_out_min{-5.0f};
    float twist_angular_z_out_max{5.0f};

    uint8_t uplink_rssi_ant1_{0};
    uint8_t uplink_rssi_ant2_{0};

    bool twist_pub_just_once_{false};
};

}  // namespace rover_crfs_telop

#endif  // ROVER_CRFS_TELEOP_HPP_