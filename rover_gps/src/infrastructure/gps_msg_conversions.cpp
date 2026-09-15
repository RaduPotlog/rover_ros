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

#include "rover_gps/infrastructure/gps_msg_conversions.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace rover_gps::infrastructure
{

namespace
{
using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;
using NavSatStatusMsg = sensor_msgs::msg::NavSatStatus;

// Roll and pitch are not estimated; this variance only keeps the covariance matrix valid.
constexpr double kUnusedAxisVariance = 1.0e3;

domain::FixStatus toFixStatus(std::int8_t status)
{
    switch (status) {
        case NavSatStatusMsg::STATUS_FIX:
            return domain::FixStatus::Fix;
        case NavSatStatusMsg::STATUS_SBAS_FIX:
            return domain::FixStatus::SbasFix;
        case NavSatStatusMsg::STATUS_GBAS_FIX:
            return domain::FixStatus::GbasFix;
        default:
            return domain::FixStatus::NoFix;
    }
}
}  // namespace

domain::GnssFix toGnssFix(const NavSatFixMsg & msg, double stamp_s)
{
    domain::GnssFix fix;
    fix.latitude_deg = msg.latitude;
    fix.longitude_deg = msg.longitude;
    fix.altitude_m = msg.altitude;
    fix.status = toFixStatus(msg.status.status);
    fix.stamp_s = stamp_s;

    if (msg.position_covariance_type != NavSatFixMsg::COVARIANCE_TYPE_UNKNOWN) {
        // The worse of the east and north variances.
        const double variance = std::max(msg.position_covariance[0], msg.position_covariance[4]);
        fix.horizontal_std_m =
            variance >= 0.0 ? std::sqrt(variance) : std::numeric_limits<double>::quiet_NaN();
    }
    return fix;
}

domain::OdometrySample toOdometrySample(const OdometryMsg & msg, double stamp_s)
{
    const auto & q = msg.pose.pose.orientation;

    domain::OdometrySample sample;
    sample.stamp_s = stamp_s;
    sample.yaw_rad = yawFromQuaternion(q.x, q.y, q.z, q.w);
    sample.vx_m_s = msg.twist.twist.linear.x;
    sample.yaw_rate_rad_s = msg.twist.twist.angular.z;
    return sample;
}

double yawFromQuaternion(double x, double y, double z, double w)
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

ImuMsg toHeadingImuMsg(
    const domain::EnuHeading & heading, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
{
    ImuMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = std::sin(0.5 * heading.yaw_rad);
    msg.orientation.w = std::cos(0.5 * heading.yaw_rad);

    msg.orientation_covariance.fill(0.0);
    msg.orientation_covariance[0] = kUnusedAxisVariance;
    msg.orientation_covariance[4] = kUnusedAxisVariance;
    msg.orientation_covariance[8] = heading.yaw_std_rad * heading.yaw_std_rad;

    msg.angular_velocity_covariance.fill(0.0);
    msg.angular_velocity_covariance[0] = -1.0;
    msg.linear_acceleration_covariance.fill(0.0);
    msg.linear_acceleration_covariance[0] = -1.0;
    return msg;
}

unsigned char toDiagnosticLevel(domain::HealthLevel level)
{
    switch (level) {
        case domain::HealthLevel::Ok:
            return DiagnosticStatusMsg::OK;
        case domain::HealthLevel::Warn:
            return DiagnosticStatusMsg::WARN;
        case domain::HealthLevel::Error:
            return DiagnosticStatusMsg::ERROR;
        default:
            return DiagnosticStatusMsg::STALE;
    }
}

const char * fixStatusText(domain::FixStatus status)
{
    switch (status) {
        case domain::FixStatus::Fix:
            return "Fix";
        case domain::FixStatus::SbasFix:
            return "SBAS fix";
        case domain::FixStatus::GbasFix:
            return "GBAS fix";
        default:
            return "No fix";
    }
}

const char * alignmentStateText(domain::AlignmentState state)
{
    switch (state) {
        case domain::AlignmentState::Collecting:
            return "Collecting";
        case domain::AlignmentState::Aligned:
            return "Aligned";
        default:
            return "Waiting for straight motion";
    }
}

}  // namespace rover_gps::infrastructure
