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

#include "rover_gps/infrastructure/nmea_msg_conversions.hpp"

#include <cmath>

#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace rover_gps::infrastructure
{

builtin_interfaces::msg::Time toRosTime(double seconds)
{
    builtin_interfaces::msg::Time stamp;
    if (!std::isfinite(seconds) || seconds < 0.0) {
        return stamp;
    }

    const double whole = std::floor(seconds);
    stamp.sec = static_cast<int32_t>(whole);
    stamp.nanosec = static_cast<uint32_t>((seconds - whole) * 1.0e9);
    return stamp;
}

int8_t toNavSatStatus(domain::FixStatus status)
{
    switch (status) {
        case domain::FixStatus::Fix:
            return sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        case domain::FixStatus::SbasFix:
            return sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        case domain::FixStatus::GbasFix:
            return sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        case domain::FixStatus::NoFix:
        default:
            return sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
}

uint8_t toNavSatCovarianceType(domain::nmea::CovarianceType type)
{
    switch (type) {
        case domain::nmea::CovarianceType::Approximated:
            return sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        case domain::nmea::CovarianceType::Unknown:
        default:
            return sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
}

NavSatFixMsg toNavSatFixMsg(
    const domain::nmea::FixOutput & fix, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
{
    NavSatFixMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.status.status = toNavSatStatus(fix.status);
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    msg.latitude = fix.latitude_deg;
    msg.longitude = fix.longitude_deg;
    msg.altitude = fix.altitude_m;

    msg.position_covariance_type = toNavSatCovarianceType(fix.covariance_type);
    for (std::size_t i = 0; i < fix.position_covariance.size(); ++i) {
        msg.position_covariance[i] = fix.position_covariance[i];
    }

    return msg;
}

TwistStampedMsg toTwistStampedMsg(
    const domain::nmea::VelocityOutput & velocity, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
{
    TwistStampedMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.twist.linear.x = velocity.east_m_s;
    msg.twist.linear.y = velocity.north_m_s;
    return msg;
}

QuaternionStampedMsg toQuaternionStampedMsg(
    const domain::nmea::HeadingOutput & heading, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
{
    QuaternionStampedMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    // HDT carries yaw only, so the quaternion is a pure rotation about z.
    const double half_yaw = heading.yaw_rad * 0.5;
    msg.quaternion.x = 0.0;
    msg.quaternion.y = 0.0;
    msg.quaternion.z = std::sin(half_yaw);
    msg.quaternion.w = std::cos(half_yaw);
    return msg;
}

TimeReferenceMsg toTimeReferenceMsg(
    const domain::nmea::TimeRefOutput & time_ref, const std::string & frame_id,
    const std::string & source, const builtin_interfaces::msg::Time & stamp)
{
    TimeReferenceMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.time_ref = toRosTime(time_ref.utc_time_s);
    // Upstream falls back to the frame id when no source is configured.
    msg.source = source.empty() ? frame_id : source;
    return msg;
}

}  // namespace rover_gps::infrastructure
