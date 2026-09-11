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

#include "rover_crfs_teleop/infrastructure/ros2_velocity_command_publisher.hpp"

namespace rover_crfs_telop
{

Ros2VelocityCommandPublisher::Ros2VelocityCommandPublisher(
    rclcpp_lifecycle::LifecycleNode & node, const std::string & topic,
    const std::string & frame_id)
: clock_(node.get_clock()),
  frame_id_(frame_id),
  // Commands are reliable/volatile (the default), per rules/ros2_communication.md.
  publisher_(node.create_publisher<geometry_msgs::msg::TwistStamped>(topic, 10))
{
}

void Ros2VelocityCommandPublisher::publish(const VelocityCommand & command)
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = clock_->now();
    msg.header.frame_id = frame_id_;
    msg.twist.linear.x = command.linear_x;
    msg.twist.angular.z = command.angular_z;

    publisher_->publish(msg);
}

void Ros2VelocityCommandPublisher::on_activate()
{
    publisher_->on_activate();
}

void Ros2VelocityCommandPublisher::on_deactivate()
{
    publisher_->on_deactivate();
}

}  // namespace rover_crfs_telop
