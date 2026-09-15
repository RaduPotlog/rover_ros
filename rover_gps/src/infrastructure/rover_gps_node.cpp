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

#include "rover_gps/infrastructure/rover_gps_node.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "rover_gps/infrastructure/ros2_gps_health_publisher.hpp"
#include "rover_gps/infrastructure/ros2_heading_publisher.hpp"

namespace rover_gps
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describePositive(
    const std::string & description, double max_value)
{
    auto descriptor = describe(description);
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 1.0e-6;
    descriptor.floating_point_range[0].to_value = max_value;
    return descriptor;
}

}  // namespace

RoverGpsNode::RoverGpsNode(
    const std::string & node_name,
    const std::string & ns,
    const rclcpp::NodeOptions & options)
: Node(node_name, ns, options)
, diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
    const domain::GpsHealthThresholds health_defaults;
    health_thresholds_.expected_rate_hz = declare_parameter(
        "expected_rate_hz", health_defaults.expected_rate_hz,
        describePositive("GGA sentence rate the receiver is configured for [Hz].", 100.0));
    health_thresholds_.min_rate_ratio = declare_parameter(
        "min_rate_ratio", health_defaults.min_rate_ratio,
        describePositive("Warn below expected_rate_hz * min_rate_ratio.", 1.0));
    health_thresholds_.fix_timeout_s = declare_parameter(
        "fix_timeout_s", health_defaults.fix_timeout_s,
        describePositive("Report an error when no fix arrives for this long [s].", 600.0));
    health_thresholds_.warn_horizontal_std_m = declare_parameter(
        "warn_horizontal_std_m", health_defaults.warn_horizontal_std_m,
        describePositive("Warn above this horizontal 1-sigma error [m].", 1.0e4));
    health_thresholds_.error_horizontal_std_m = declare_parameter(
        "error_horizontal_std_m", health_defaults.error_horizontal_std_m,
        describePositive("Report an error above this horizontal 1-sigma error [m].", 1.0e4));

    heading_settings_.publish_heading = declare_parameter(
        "publish_heading", false,
        describe("Publish gps/heading_imu once aligned (true when GPS is fused by the EKF)."));
    heading_settings_.min_heading_std_rad = declare_parameter(
        "min_heading_std_rad", 0.05,
        describePositive("Lower bound of the published heading standard deviation [rad].", 3.2));
    heading_frame_id_ = declare_parameter(
        "heading_frame_id", std::string("base_link"),
        describe("frame_id of gps/heading_imu; the heading is the orientation of this frame."));

    const domain::HeadingAlignmentConfig alignment_defaults;
    alignment_config_.min_segment_length_m = declare_parameter(
        "alignment.min_segment_length_m", alignment_defaults.min_segment_length_m,
        describePositive("Straight distance that makes one course measurement [m].", 1000.0));
    alignment_config_.min_speed_m_s = declare_parameter(
        "alignment.min_speed_m_s", alignment_defaults.min_speed_m_s,
        describePositive("Minimum |forward speed| while measuring [m/s].", 10.0));
    alignment_config_.max_yaw_rate_rad_s = declare_parameter(
        "alignment.max_yaw_rate_rad_s", alignment_defaults.max_yaw_rate_rad_s,
        describePositive("Maximum |yaw rate| while measuring [rad/s].", 10.0));
    alignment_config_.max_yaw_change_rad = declare_parameter(
        "alignment.max_yaw_change_rad", alignment_defaults.max_yaw_change_rad,
        describePositive("Maximum yaw change within one segment [rad].", 3.2));
    alignment_config_.max_horizontal_std_m = declare_parameter(
        "alignment.max_horizontal_std_m", alignment_defaults.max_horizontal_std_m,
        describePositive("Ignore fixes with a larger horizontal 1-sigma error [m].", 1.0e4));
    alignment_config_.max_fix_gap_s = declare_parameter(
        "alignment.max_fix_gap_s", alignment_defaults.max_fix_gap_s,
        describePositive("Restart the segment after a longer gap between fixes [s].", 600.0));
    alignment_config_.max_odometry_age_s = declare_parameter(
        "alignment.max_odometry_age_s", alignment_defaults.max_odometry_age_s,
        describePositive("Ignore a fix when the latest odometry is older than this [s].", 60.0));
    alignment_config_.max_offset_std_rad = declare_parameter(
        "alignment.max_offset_std_rad", alignment_defaults.max_offset_std_rad,
        describePositive("Segment offsets must agree within this circular std [rad].", 3.2));

    auto segments_descriptor = describe("Accepted segments needed before aligning.");
    segments_descriptor.integer_range.resize(1);
    segments_descriptor.integer_range[0].from_value = 1;
    segments_descriptor.integer_range[0].to_value = 100;
    alignment_config_.required_segments = static_cast<std::size_t>(declare_parameter<std::int64_t>(
        "alignment.required_segments",
        static_cast<std::int64_t>(alignment_defaults.required_segments), segments_descriptor));

    // Reject inconsistent combinations (e.g. warn > error) at startup, not at the first fix.
    domain::GpsHealthEvaluator::validate(health_thresholds_);
    domain::HeadingAlignmentEstimator::validate(alignment_config_);
}

void RoverGpsNode::init()
{
    diagnostic_updater_->setHardwareID("RoverGps");

    monitor_gps_ = std::make_unique<application::MonitorGpsUseCase>(
        std::make_shared<infrastructure::Ros2GpsHealthPublisher>(diagnostic_updater_),
        health_thresholds_);

    align_heading_ = std::make_unique<application::AlignHeadingUseCase>(
        std::make_shared<infrastructure::Ros2HeadingPublisher>(
            *this, diagnostic_updater_, heading_frame_id_, heading_settings_.publish_heading),
        alignment_config_, heading_settings_);

    fix_subscriber_ = create_subscription<infrastructure::NavSatFixMsg>(
        "gps/fix", rclcpp::SensorDataQoS(), std::bind(&RoverGpsNode::fixCallback, this, _1));

    odometry_subscriber_ = create_subscription<infrastructure::OdometryMsg>(
        "odom", rclcpp::QoS(10), std::bind(&RoverGpsNode::odometryCallback, this, _1));

    reset_alignment_service_ = create_service<std_srvs::srv::Trigger>(
        "gps/reset_heading_alignment",
        std::bind(&RoverGpsNode::resetAlignmentCallback, this, _1, _2));

    tick_timer_ = create_wall_timer(1s, std::bind(&RoverGpsNode::tickCallback, this));

    // Publish STALE/unaligned statuses immediately instead of after the first timer period.
    tickCallback();
}

void RoverGpsNode::fixCallback(const infrastructure::NavSatFixMsg::SharedPtr msg)
{
    const domain::GnssFix fix = infrastructure::toGnssFix(*msg, nowSeconds());
    monitor_gps_->onFix(fix);

    const bool was_aligned = align_heading_->aligned();
    align_heading_->onFix(fix);
    if (!was_aligned && align_heading_->aligned()) {
        // Report (and log) the alignment now rather than at the next tick.
        align_heading_->onTick();
    }
}

void RoverGpsNode::odometryCallback(const infrastructure::OdometryMsg::SharedPtr msg)
{
    align_heading_->onOdometry(infrastructure::toOdometrySample(*msg, nowSeconds()));
}

void RoverGpsNode::tickCallback()
{
    monitor_gps_->onTick(nowSeconds());
    align_heading_->onTick();
}

void RoverGpsNode::resetAlignmentCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    align_heading_->reset();
    response->success = true;
    response->message = "Heading alignment reset; drive straight to realign.";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

double RoverGpsNode::nowSeconds() const
{
    return get_clock()->now().seconds();
}

}  // namespace rover_gps
