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

#ifndef ROVER_CRFS_TELEOP_INFRASTRUCTURE_ROS2_VELOCITY_COMMAND_PUBLISHER_HPP_
#define ROVER_CRFS_TELEOP_INFRASTRUCTURE_ROS2_VELOCITY_COMMAND_PUBLISHER_HPP_

#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "rover_crfs_teleop/domain/ports.hpp"

namespace rover_crfs_teleop
{

// Publishes teleop velocity commands as TwistStamped for twist_mux. Lifecycle-managed: it only
// publishes between the owning node's activate and deactivate.
class Ros2VelocityCommandPublisher : public VelocityCommandPort
{

public:

    Ros2VelocityCommandPublisher(
        rclcpp_lifecycle::LifecycleNode & node, const std::string & topic,
        const std::string & frame_id);

    void publish(const VelocityCommand & command) override;

    void on_activate();

    void on_deactivate();

private:

    rclcpp::Clock::SharedPtr clock_;
    std::string frame_id_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

}  // namespace rover_crfs_teleop

#endif  // ROVER_CRFS_TELEOP_INFRASTRUCTURE_ROS2_VELOCITY_COMMAND_PUBLISHER_HPP_
