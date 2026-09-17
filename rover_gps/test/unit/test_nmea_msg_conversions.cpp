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
#include <limits>

#include "sensor_msgs/msg/nav_sat_status.hpp"

#include "rover_gps/infrastructure/nmea_msg_conversions.hpp"

using namespace rover_gps::domain;              // NOLINT
using namespace rover_gps::domain::nmea;         // NOLINT
using namespace rover_gps::infrastructure;       // NOLINT

namespace
{

constexpr double kPi = 3.14159265358979323846;

builtin_interfaces::msg::Time makeStamp(int32_t sec, uint32_t nanosec)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = sec;
    stamp.nanosec = nanosec;
    return stamp;
}

}  // namespace

TEST(NmeaMsgConversionsTest, ConvertsSecondsToRosTime)
{
    const auto stamp = toRosTime(1758112519.25);
    EXPECT_EQ(stamp.sec, 1758112519);
    EXPECT_NEAR(static_cast<double>(stamp.nanosec), 0.25e9, 1.0e3);
}

TEST(NmeaMsgConversionsTest, NonFiniteOrNegativeSecondsBecomeZero)
{
    for (const double seconds : {std::nan(""), -1.0, std::numeric_limits<double>::infinity()}) {
        const auto stamp = toRosTime(seconds);
        EXPECT_EQ(stamp.sec, 0);
        EXPECT_EQ(stamp.nanosec, 0u);
    }
}

TEST(NmeaMsgConversionsTest, MapsFixStatusOntoNavSatStatus)
{
    EXPECT_EQ(toNavSatStatus(FixStatus::NoFix), sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
    EXPECT_EQ(toNavSatStatus(FixStatus::Fix), sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    EXPECT_EQ(toNavSatStatus(FixStatus::SbasFix), sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX);
    EXPECT_EQ(toNavSatStatus(FixStatus::GbasFix), sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX);
}

TEST(NmeaMsgConversionsTest, MapsCovarianceType)
{
    EXPECT_EQ(
        toNavSatCovarianceType(CovarianceType::Unknown),
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
    EXPECT_EQ(
        toNavSatCovarianceType(CovarianceType::Approximated),
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED);
}

TEST(NmeaMsgConversionsTest, FillsNavSatFix)
{
    FixOutput fix;
    fix.latitude_deg = 48.1173;
    fix.longitude_deg = 11.516667;
    fix.altitude_m = 592.3;
    fix.status = FixStatus::Fix;
    fix.covariance_type = CovarianceType::Approximated;
    fix.position_covariance[0] = 12.96;
    fix.position_covariance[4] = 12.96;
    fix.position_covariance[8] = 207.36;

    const auto msg = toNavSatFixMsg(fix, "gps_link", makeStamp(5, 250000000));

    EXPECT_EQ(msg.header.frame_id, "gps_link");
    EXPECT_EQ(msg.header.stamp.sec, 5);
    EXPECT_EQ(msg.header.stamp.nanosec, 250000000u);
    EXPECT_EQ(msg.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    EXPECT_EQ(msg.status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
    EXPECT_NEAR(msg.latitude, 48.1173, 1e-9);
    EXPECT_NEAR(msg.longitude, 11.516667, 1e-9);
    EXPECT_NEAR(msg.altitude, 592.3, 1e-9);
    EXPECT_EQ(msg.position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED);
    EXPECT_NEAR(msg.position_covariance[0], 12.96, 1e-9);
    EXPECT_NEAR(msg.position_covariance[4], 12.96, 1e-9);
    EXPECT_NEAR(msg.position_covariance[8], 207.36, 1e-9);
}

TEST(NmeaMsgConversionsTest, VelocityKeepsTheUpstreamAxisConvention)
{
    // east lands on linear.x and north on linear.y, as the Python driver did.
    const auto msg = toTwistStampedMsg(VelocityOutput{2.5, 1.5}, "gps_link", makeStamp(1, 0));

    EXPECT_EQ(msg.header.frame_id, "gps_link");
    EXPECT_NEAR(msg.twist.linear.x, 2.5, 1e-9);
    EXPECT_NEAR(msg.twist.linear.y, 1.5, 1e-9);
    EXPECT_NEAR(msg.twist.linear.z, 0.0, 1e-12);
    EXPECT_NEAR(msg.twist.angular.z, 0.0, 1e-12);
}

TEST(NmeaMsgConversionsTest, HeadingBecomesAPureYawQuaternion)
{
    const auto msg = toQuaternionStampedMsg(HeadingOutput{kPi / 2.0}, "gps_link", makeStamp(1, 0));

    EXPECT_NEAR(msg.quaternion.x, 0.0, 1e-12);
    EXPECT_NEAR(msg.quaternion.y, 0.0, 1e-12);
    EXPECT_NEAR(msg.quaternion.z, std::sin(kPi / 4.0), 1e-9);
    EXPECT_NEAR(msg.quaternion.w, std::cos(kPi / 4.0), 1e-9);

    // ... and stays normalised.
    const double norm = std::sqrt(
        msg.quaternion.x * msg.quaternion.x + msg.quaternion.y * msg.quaternion.y +
        msg.quaternion.z * msg.quaternion.z + msg.quaternion.w * msg.quaternion.w);
    EXPECT_NEAR(norm, 1.0, 1e-12);
}

TEST(NmeaMsgConversionsTest, TimeReferenceCarriesReceiverUtc)
{
    const auto msg =
        toTimeReferenceMsg(TimeRefOutput{1758112519.0}, "gps_link", "gps", makeStamp(9, 0));

    EXPECT_EQ(msg.header.frame_id, "gps_link");
    EXPECT_EQ(msg.header.stamp.sec, 9);
    EXPECT_EQ(msg.time_ref.sec, 1758112519);
    EXPECT_EQ(msg.source, "gps");
}

TEST(NmeaMsgConversionsTest, TimeReferenceFallsBackToTheFrameIdAsSource)
{
    const auto msg =
        toTimeReferenceMsg(TimeRefOutput{1758112519.0}, "gps_link", "", makeStamp(9, 0));
    EXPECT_EQ(msg.source, "gps_link");
}
