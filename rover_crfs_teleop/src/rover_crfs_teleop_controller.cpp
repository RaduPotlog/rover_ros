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

#include "rover_crfs_teleop/rover_crfs_teleop_controller.hpp"

namespace rover_crfs_telop
{

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
        "/rc/channels", rclcpp::QoS(1).best_effort().durability_volatile(), topic_callback);

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/teleop_elrs_cmd_vel_stamped", 10
    );

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
}

} // rover_crfs_telop