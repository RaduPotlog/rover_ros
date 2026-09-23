// Copyright 2026 Mechatronics Academy
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

#include "rover_twist_mux/infrastructure/command_freshness_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace rover_twist_mux
{

namespace
{

// A drop within this long ago [s] keeps the diagnostic at WARN, so a burst is visible on a
// dashboard refreshed at 1 Hz rather than only in the log.
constexpr double kRecentDropWindowS = 5.0;

double wallNowSeconds()
{
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

domain::CommandFreshnessConfig toConfig(const command_freshness::Params & params)
{
    domain::CommandFreshnessConfig config;

    config.max_delay_s = params.max_delay;
    config.max_clock_drift = params.max_clock_drift;
    config.resync_time_s = params.resync_time;
    config.resync_gap_s = params.resync_gap;

    return config;
}

const char * toString(domain::FreshnessVerdict verdict)
{
    switch (verdict) {
        case domain::FreshnessVerdict::Fresh: return "fresh";
        case domain::FreshnessVerdict::Resynced: return "resynced";
        case domain::FreshnessVerdict::Stale: return "stale";
        case domain::FreshnessVerdict::Unstamped: return "unstamped";
        default: return "unknown";
    }
}

}  // namespace

CommandFreshnessNode::CommandFreshnessNode(
    const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
, diagnostic_updater_(this)
{
    param_listener_ = std::make_shared<command_freshness::ParamListener>(
        this->get_node_parameters_interface());

    const auto params = param_listener_->get_params();

    filter_ = std::make_unique<domain::CommandFreshnessFilter>(toConfig(params));

    // Reliable both ways: foxglove_bridge's client publisher and twist_mux's SystemDefaultsQoS
    // subscription are reliable, and a best-effort hop here would reintroduce the lost-zero risk
    // the Driver UI's zero burst covers.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    command_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(params.output_topic, qos);

    command_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        params.input_topic, qos,
        std::bind(&CommandFreshnessNode::commandCallback, this, std::placeholders::_1));

    diagnostic_updater_.setHardwareID("Command Freshness");
    diagnostic_updater_.add("Command freshness", this, &CommandFreshnessNode::diagnoseFreshness);

    RCLCPP_INFO(
        this->get_logger(),
        "Passing commands from '%s' to '%s' while at most %.2f s later than the best recent "
        "delivery.",
        command_sub_->get_topic_name(), command_pub_->get_topic_name(), params.max_delay);
}

void CommandFreshnessNode::commandCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    const auto verdict = filter_->accept(stampSeconds(msg->header.stamp), wallNowSeconds());

    if (domain::isAccepted(verdict)) {
        if (verdict == domain::FreshnessVerdict::Resynced) {
            RCLCPP_WARN(
                this->get_logger(),
                "Re-baselined on a steady run of late commands (sender clock stepped?); "
                "passing commands again.");
        }
        command_pub_->publish(*msg);
        return;
    }

    const auto & stats = filter_->stats();
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropping %s command: %.2f s later than the best recent delivery (%lu dropped so far).",
        toString(verdict), stats.last_excess_delay_s, static_cast<unsigned long>(stats.rejected));
}

void CommandFreshnessNode::diagnoseFreshness(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    using diagnostic_msgs::msg::DiagnosticStatus;

    const auto & stats = filter_->stats();
    const auto & config = filter_->config();

    status.add("Accepted", stats.accepted);
    status.add("Dropped", stats.rejected);
    status.add("Resyncs", stats.resyncs);
    if (stats.baseline_s.has_value()) {
        status.add("Latency baseline (s)", *stats.baseline_s);
    }
    status.add("Last excess delay (s)", stats.last_excess_delay_s);
    if (stats.last_verdict.has_value()) {
        status.add("Last verdict", toString(*stats.last_verdict));
    }
    status.add("Max delay (s)", config.max_delay_s);

    if (!stats.last_verdict.has_value()) {
        status.summary(DiagnosticStatus::OK, "No commands received yet.");
        return;
    }

    const bool dropped_recently = stats.last_rejected_at_s.has_value() &&
        wallNowSeconds() - *stats.last_rejected_at_s < kRecentDropWindowS;

    if (dropped_recently) {
        status.summary(
            DiagnosticStatus::WARN,
            "Dropping late commands (" + std::to_string(stats.last_excess_delay_s) +
            " s late): the link to the Driver UI is stalling, or its clock stepped.");
        return;
    }

    status.summary(DiagnosticStatus::OK, "Passing fresh commands.");
}

}  // namespace rover_twist_mux
