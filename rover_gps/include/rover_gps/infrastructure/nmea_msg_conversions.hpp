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

#ifndef ROVER_GPS_INFRASTRUCTURE_NMEA_MSG_CONVERSIONS_HPP_
#define ROVER_GPS_INFRASTRUCTURE_NMEA_MSG_CONVERSIONS_HPP_

#include <cstdint>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

#include "rover_gps/domain/nmea/nmea_fix_assembler.hpp"

namespace rover_gps::infrastructure
{

// Repeats the alias from gps_msg_conversions.hpp; both name the same type, so including both
// headers in one translation unit is fine.
using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
using QuaternionStampedMsg = geometry_msgs::msg::QuaternionStamped;
using TimeReferenceMsg = sensor_msgs::msg::TimeReference;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

/** @brief Seconds since an epoch to a ROS time message. */
builtin_interfaces::msg::Time toRosTime(double seconds);

/** @brief Maps a domain fix status onto sensor_msgs/NavSatStatus STATUS_*. */
int8_t toNavSatStatus(domain::FixStatus status);

/** @brief Maps a domain covariance type onto sensor_msgs/NavSatFix COVARIANCE_TYPE_*. */
uint8_t toNavSatCovarianceType(domain::nmea::CovarianceType type);

NavSatFixMsg toNavSatFixMsg(
    const domain::nmea::FixOutput & fix, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp);

/** @note `east_m_s` lands in `twist.linear.x` and `north_m_s` in `twist.linear.y`, preserving the
 *        upstream driver's axis convention. */
TwistStampedMsg toTwistStampedMsg(
    const domain::nmea::VelocityOutput & velocity, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp);

/** @brief Pure-yaw quaternion from the HDT heading. */
QuaternionStampedMsg toQuaternionStampedMsg(
    const domain::nmea::HeadingOutput & heading, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp);

TimeReferenceMsg toTimeReferenceMsg(
    const domain::nmea::TimeRefOutput & time_ref, const std::string & frame_id,
    const std::string & source, const builtin_interfaces::msg::Time & stamp);

}  // namespace rover_gps::infrastructure

#endif  // ROVER_GPS_INFRASTRUCTURE_NMEA_MSG_CONVERSIONS_HPP_
