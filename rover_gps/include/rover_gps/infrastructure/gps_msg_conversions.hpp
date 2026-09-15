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

#ifndef ROVER_GPS_INFRASTRUCTURE_GPS_MSG_CONVERSIONS_HPP_
#define ROVER_GPS_INFRASTRUCTURE_GPS_MSG_CONVERSIONS_HPP_

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rover_gps/domain/gnss_fix.hpp"
#include "rover_gps/domain/gps_health_evaluator.hpp"
#include "rover_gps/domain/heading_alignment_estimator.hpp"
#include "rover_gps/domain/ports/heading_publisher_port.hpp"

namespace rover_gps::infrastructure
{

using ImuMsg = sensor_msgs::msg::Imu;
using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
using OdometryMsg = nav_msgs::msg::Odometry;

/** @param stamp_s receive time, on the clock used for odometry samples too. */
domain::GnssFix toGnssFix(const NavSatFixMsg & msg, double stamp_s);

/** @param stamp_s receive time, on the clock used for GNSS fixes too. */
domain::OdometrySample toOdometrySample(const OdometryMsg & msg, double stamp_s);

double yawFromQuaternion(double x, double y, double z, double w);

/**
 * @brief Orientation-only IMU message for navsat_transform_node: roll = pitch = 0, the ENU yaw,
 *        angular velocity and linear acceleration flagged unused (covariance[0] = -1).
 */
ImuMsg toHeadingImuMsg(
    const domain::EnuHeading & heading, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp);

unsigned char toDiagnosticLevel(domain::HealthLevel level);

const char * fixStatusText(domain::FixStatus status);

const char * alignmentStateText(domain::AlignmentState state);

}  // namespace rover_gps::infrastructure

#endif  // ROVER_GPS_INFRASTRUCTURE_GPS_MSG_CONVERSIONS_HPP_
