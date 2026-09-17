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

#ifndef ROVER_CRSF_TELEOP_INFRASTRUCTURE_ROVER_CRSF_TELEOP_NODE_HPP_
#define ROVER_CRSF_TELEOP_INFRASTRUCTURE_ROVER_CRSF_TELEOP_NODE_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "rover_crsf_teleop/application/teleop_use_case.hpp"
#include "rover_crsf_teleop/domain/crsf/crsf_parser.hpp"
#include "rover_crsf_teleop/infrastructure/rc_message_conversions.hpp"
#include "rover_crsf_teleop/infrastructure/ros2_trigger_safety_switch.hpp"
#include "rover_crsf_teleop/infrastructure/ros2_velocity_command_publisher.hpp"

namespace rover_crsf_teleop
{

// ROS adapter for TeleopUseCase: raw CRSF bytes in, teleop_elrs_cmd_vel_stamped and the hardware
// interface's E-Stop services out.
//
// The UART itself belongs to rover_serial_driver's rover_serial_bridge_node node, which this package's launch
// file starts; this node subscribes to the byte stream that node publishes and decodes CRSF
// from it. Decoding here rather than in a separate process means the decode runs on the
// executor thread - the same thread as the control timer - so TeleopUseCase keeps its
// single-threaded invariant with no locking at all.
//
// Lifecycle-managed, so a supervisor can take RC teleop off the command path (deactivate)
// without killing the process:
//   - configure:  read parameters, create the subscription, adapters and the use case;
//   - activate:   start the 20 ms control timer;
//   - deactivate: publish one zero command, then stop the timer.
//
// Designed for a single-threaded executor: the subscription and the timer both touch the use
// case, and it is only safe because they never run concurrently. The diagnostic_updater timer
// runs on the same executor.
//
// Diagnostics (hardware ID "RC Receiver"), published in every lifecycle state:
//   - "RC link":          the LinkMonitor verdict that gates the command, with ages / LQ / switches
//                         (WARN while waiting for the first frame and while the link is lost);
//   - "RC serial link":   whether the byte stream from rover_serial_bridge_node is arriving and decoding
//                         (WARN when it is silent - which is how a dead bridge becomes visible);
//   - "E-Stop requests":  reachability and last outcome of the hardware interface E-Stop services;
//   - "RC channels rate": decoded CRSF frame rate against rc_channels_expected_hz (WARN at
//                         worst, also when no frames arrive).
// None of them reports ERROR: RC teleop is optional.
class RoverCrsfTeleopNode : public rclcpp_lifecycle::LifecycleNode, private crsf::CrsfSink
{

public:

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit RoverCrsfTeleopNode(
        const std::string & node_name = "rover_crsf_teleop_node",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:

    // crsf::CrsfSink - called synchronously from parser_.parse(), on the executor thread.
    void onRcChannels(const RcFrame & frame) override;

    void onLinkStatistics(const RcLinkStats & stats) override;

    // Declared once, in the constructor, so a cleanup -> configure cycle doesn't re-declare them.
    void declareParameters();

    // Reads the parameters into a TeleopConfig, or returns nullopt (after logging why) when they
    // are inconsistent.
    std::optional<TeleopConfig> readConfig();

    void controlTimerCallback();

    void releaseResources();

    void diagnoseRcLink(diagnostic_updater::DiagnosticStatusWrapper & status);

    void diagnoseSafetyRequests(diagnostic_updater::DiagnosticStatusWrapper & status);

    void diagnoseChannelsRate(diagnostic_updater::DiagnosticStatusWrapper & status);

    void diagnoseSerialLink(diagnostic_updater::DiagnosticStatusWrapper & status);

    std::unique_ptr<TeleopUseCase> use_case_;
    std::shared_ptr<Ros2VelocityCommandPublisher> velocity_publisher_;
    std::shared_ptr<Ros2TriggerSafetySwitch> safety_switch_;

    // Raw CRSF bytes from rover_serial_bridge_node.
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_subscriber_;

    crsf::CrsfParser parser_;

    // Plain publishers, deliberately not lifecycle ones: these are observability echoes, and a
    // LifecyclePublisher would silently drop them exactly when the node is deactivated to work
    // out why it was deactivated. Only the cmd_vel publisher is lifecycle-gated.
    rclcpp::Publisher<rover_msgs::msg::RcChannels>::SharedPtr rc_channels_publisher_;
    rclcpp::Publisher<rover_msgs::msg::RcLinkStatus>::SharedPtr rc_link_publisher_;

    bool publish_rc_topics_{true};

    // For the "RC serial link" diagnostic: how the byte stream itself is doing, as distinct from
    // whether the RC link carries a usable signal.
    std::optional<SteadyTime> last_serial_message_;
    std::optional<SteadyTime> last_decoded_frame_;
    std::uint64_t serial_bytes_received_{0};
    std::uint64_t decoded_frames_{0};
    std::uint64_t decoded_link_stats_{0};

    rclcpp::TimerBase::SharedPtr control_timer_;

    std::optional<TickStatus> last_tick_status_;

    // FrequencyStatusParam holds pointers to these, so they must outlive channels_rate_.
    double channels_min_hz_{0.0};
    double channels_max_hz_{0.0};
    std::unique_ptr<diagnostic_updater::FrequencyStatus> channels_rate_;

    // Last member: destroyed first, so its timer never runs a task on a half-destroyed node.
    std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_INFRASTRUCTURE_ROVER_CRSF_TELEOP_NODE_HPP_
