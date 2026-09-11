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

#ifndef ROVER_CRFS_TELEOP_INFRASTRUCTURE_ROVER_CRFS_TELEOP_NODE_HPP_
#define ROVER_CRFS_TELEOP_INFRASTRUCTURE_ROVER_CRFS_TELEOP_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "crsf_receiver_msg/msg/crsf_channels16.hpp"
#include "crsf_receiver_msg/msg/crsf_link_info.hpp"

#include "rover_crfs_teleop/application/teleop_use_case.hpp"
#include "rover_crfs_teleop/infrastructure/ros2_trigger_safety_switch.hpp"
#include "rover_crfs_teleop/infrastructure/ros2_velocity_command_publisher.hpp"

namespace rover_crfs_teleop
{

// ROS adapter for TeleopUseCase: rc/channels + rc/link in, teleop_elrs_cmd_vel_stamped and the
// hardware interface's E-Stop services out.
//
// Lifecycle-managed, so a supervisor can take RC teleop off the command path (deactivate)
// without killing the process:
//   - configure:  read parameters, create subscriptions, adapters and the use case;
//   - activate:   start the 20 ms control timer;
//   - deactivate: publish one zero command, then stop the timer.
//
// Designed for a single-threaded executor: the subscriptions and the timer all touch the use
// case, and it is only safe because they never run concurrently.
class RoverCrfsTeleopNode : public rclcpp_lifecycle::LifecycleNode
{

public:

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit RoverCrfsTeleopNode(
        const std::string & node_name = "rover_crfs_teleop_node",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:

    // Declared once, in the constructor, so a cleanup -> configure cycle doesn't re-declare them.
    void declareParameters();

    // Reads the parameters into a TeleopConfig, or returns nullopt (after logging why) when they
    // are inconsistent.
    std::optional<TeleopConfig> readConfig();

    void controlTimerCallback();

    void releaseResources();

    std::unique_ptr<TeleopUseCase> use_case_;
    std::shared_ptr<Ros2VelocityCommandPublisher> velocity_publisher_;
    std::shared_ptr<Ros2TriggerSafetySwitch> safety_switch_;

    rclcpp::Subscription<crsf_receiver_msg::msg::CRSFChannels16>::SharedPtr channels_subscriber_;
    rclcpp::Subscription<crsf_receiver_msg::msg::CRSFLinkInfo>::SharedPtr link_subscriber_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    std::optional<TickStatus> last_tick_status_;
};

}  // namespace rover_crfs_teleop

#endif  // ROVER_CRFS_TELEOP_INFRASTRUCTURE_ROVER_CRFS_TELEOP_NODE_HPP_
