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

#include "rover_gps/infrastructure/ros2_heading_publisher.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace rover_gps::infrastructure
{

namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
}  // namespace

Ros2HeadingPublisher::Ros2HeadingPublisher(
    rclcpp::Node & node,
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
    std::string frame_id, bool heading_output_enabled)
: logger_(node.get_logger())
, clock_(node.get_clock())
, frame_id_(std::move(frame_id))
, heading_output_enabled_(heading_output_enabled)
{
    heading_pub_ = node.create_publisher<ImuMsg>("gps/heading_imu", 10);

    diagnostic_updater->add("Heading alignment", this, &Ros2HeadingPublisher::diagnose);
}

void Ros2HeadingPublisher::publishHeading(const domain::EnuHeading & heading)
{
    heading_pub_->publish(toHeadingImuMsg(heading, frame_id_, clock_->now()));
}

void Ros2HeadingPublisher::publishAlignmentStatus(const domain::AlignmentStatus & status)
{
    const bool aligned = status.state == domain::AlignmentState::Aligned;
    if (aligned && !announced_alignment_ && status.yaw_offset_rad) {
        RCLCPP_INFO(logger_, "GNSS heading aligned: odom->ENU yaw offset %.1f deg (std %.1f deg).",
            *status.yaw_offset_rad * kRadToDeg, status.offset_std_rad.value_or(0.0) * kRadToDeg);
    }
    announced_alignment_ = aligned;
    status_ = status;
}

void Ros2HeadingPublisher::diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    status.add("State", alignmentStateText(status_.state));
    status.add("Accepted segments", status_.accepted_segments);
    status.add("Heading output", heading_output_enabled_ ? "enabled" : "disabled");
    if (status_.yaw_offset_rad) {
        status.addf("Yaw offset (deg)", "%.1f", *status_.yaw_offset_rad * kRadToDeg);
        status.addf("Offset std (deg)", "%.1f", status_.offset_std_rad.value_or(0.0) * kRadToDeg);
    }

    if (status_.state == domain::AlignmentState::Aligned) {
        status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Heading aligned to ENU.");
    } else if (!heading_output_enabled_) {
        // GPS is not fused: nothing waits for the heading, so an unaligned state is not a fault.
        status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK,
            "Heading not aligned; GPS fusion disabled.");
    } else {
        status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN,
            "Drive straight to align the GNSS heading.");
    }
}

}  // namespace rover_gps::infrastructure
