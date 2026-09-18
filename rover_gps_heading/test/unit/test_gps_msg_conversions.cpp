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

#include <gtest/gtest.h>

#include <cmath>

#include "sensor_msgs/msg/nav_sat_status.hpp"

#include "rover_gps_heading/infrastructure/gps_msg_conversions.hpp"

using namespace rover_gps_heading::infrastructure;  // NOLINT
using rover_gps_heading::domain::EnuHeading;
using rover_gps_heading::domain::FixStatus;
using NavSatStatusMsg = sensor_msgs::msg::NavSatStatus;

TEST(GpsMsgConversionsTest, NavSatFixToGnssFix)
{
    NavSatFixMsg msg;
    msg.latitude = 45.5;
    msg.longitude = 25.25;
    msg.altitude = 300.0;
    msg.status.status = NavSatStatusMsg::STATUS_SBAS_FIX;
    msg.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_APPROXIMATED;
    msg.position_covariance[0] = 4.0;
    msg.position_covariance[4] = 9.0;

    const auto fix = toGnssFix(msg, 12.5);
    EXPECT_DOUBLE_EQ(fix.latitude_deg, 45.5);
    EXPECT_DOUBLE_EQ(fix.longitude_deg, 25.25);
    EXPECT_DOUBLE_EQ(fix.altitude_m, 300.0);
    EXPECT_EQ(fix.status, FixStatus::SbasFix);
    EXPECT_DOUBLE_EQ(fix.horizontal_std_m, 3.0);
    EXPECT_DOUBLE_EQ(fix.stamp_s, 12.5);
}

TEST(GpsMsgConversionsTest, UnknownCovarianceAndNoFix)
{
    NavSatFixMsg msg;
    msg.status.status = NavSatStatusMsg::STATUS_NO_FIX;
    msg.position_covariance_type = NavSatFixMsg::COVARIANCE_TYPE_UNKNOWN;

    const auto fix = toGnssFix(msg, 0.0);
    EXPECT_EQ(fix.status, FixStatus::NoFix);
    EXPECT_TRUE(std::isnan(fix.horizontal_std_m));
}

TEST(GpsMsgConversionsTest, OdometryToSample)
{
    OdometryMsg msg;
    msg.pose.pose.orientation.z = std::sin(0.3);
    msg.pose.pose.orientation.w = std::cos(0.3);
    msg.twist.twist.linear.x = -0.7;
    msg.twist.twist.angular.z = 0.05;

    const auto sample = toOdometrySample(msg, 3.0);
    EXPECT_NEAR(sample.yaw_rad, 0.6, 1e-12);
    EXPECT_DOUBLE_EQ(sample.vx_m_s, -0.7);
    EXPECT_DOUBLE_EQ(sample.yaw_rate_rad_s, 0.05);
    EXPECT_DOUBLE_EQ(sample.stamp_s, 3.0);
}

TEST(GpsMsgConversionsTest, HeadingImuMessage)
{
    EnuHeading heading;
    heading.yaw_rad = 1.2;
    heading.yaw_std_rad = 0.1;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = 7;

    const ImuMsg msg = toHeadingImuMsg(heading, "rover/base_link", stamp);
    EXPECT_EQ(msg.header.frame_id, "rover/base_link");
    EXPECT_EQ(msg.header.stamp.sec, 7);
    EXPECT_NEAR(
        yawFromQuaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
        1.2, 1e-12);
    EXPECT_NEAR(msg.orientation_covariance[8], 0.01, 1e-12);
    EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[0], -1.0);
    EXPECT_DOUBLE_EQ(msg.linear_acceleration_covariance[0], -1.0);
}
