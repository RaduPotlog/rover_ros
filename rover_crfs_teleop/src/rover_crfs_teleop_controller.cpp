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

    auto topic_callback = [this](crsf_receiver_msg::msg::CRSFChannels16::UniquePtr msg) -> void {

        channels_values_.ch1 = msg->ch1;
        channels_values_.ch3 = msg->ch3;
        channels_values_.ch4 = msg->ch4;
        channels_values_.ch5 = msg->ch5;
    };
    
    subscription_ = this->create_subscription<crsf_receiver_msg::msg::CRSFChannels16>(
        "rc/channels", rclcpp::QoS(1).best_effort().durability_volatile(), topic_callback);

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
}

void RoverCrfsTeleopController::main_timer_callback()
{
    auto msg = geometry_msgs::msg::TwistStamped();
    
    float twist_linear_x = (static_cast<float>(channels_values_.ch3) - 172.0f) * (1.0f - (-1.0f)) / (1900.0f - 172.0f) + (-1.0f); 
    float twist_angular_z = (static_cast<float>(channels_values_.ch1) - 172.0f) * (1.0f - (-1.0f)) / (1900.0f - 172.0f) + (-1.0f); 

    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = twist_linear_x;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = twist_angular_z;

    cmd_vel_publisher_->publish(msg);

    unsigned int e_stop_current_value = channels_values_.ch5;
    bool e_stop_triggered = false;

    if (e_stop_current_value != e_stop_old_value) {
        e_stop_old_value = e_stop_current_value;
        e_stop_triggered = true;
    }

    if (e_stop_triggered) {
        if (e_stop_current_value < 500) {
            
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            
            while (!e_stop_set_set_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    return;
                }
                
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }

            e_stop_set_set_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
            });

        } else {
                
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            
            while (!e_stop_set_reset_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    return;
                }
                
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }

            e_stop_set_reset_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
            });
        }
    }
    
    unsigned int e_stop_latch_reset_current_value = channels_values_.ch4;
    bool e_stop_latch_reset_triggered = false;

    if (!e_stop_latch_reset_init) {
        e_stop_latch_reset_old_value = e_stop_latch_reset_current_value;

        if (e_stop_latch_reset_init_counter > 0) {
            e_stop_latch_reset_init_counter--;
        } else {
            e_stop_latch_reset_init = true;
        }
    }

    if (e_stop_latch_reset_current_value != e_stop_latch_reset_old_value) {
        e_stop_latch_reset_old_value = e_stop_latch_reset_current_value;
        e_stop_latch_reset_triggered = true;
    }

    if (e_stop_latch_reset_triggered) {
        if (e_stop_latch_reset_current_value < 500) {
            
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            
            while (!sw_e_stop_latch_reset_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    return;
                }
                
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }

            sw_e_stop_latch_reset_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
            });

        }
    }
}

} // rover_crfs_telop