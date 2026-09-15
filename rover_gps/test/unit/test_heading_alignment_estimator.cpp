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
#include <stdexcept>

#include "rover_gps/domain/geo_math.hpp"
#include "rover_gps/domain/heading_alignment_estimator.hpp"

using namespace rover_gps::domain;  // NOLINT

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kLat0 = 45.0;
constexpr double kLon0 = 25.0;

/** @brief Drives a simulated rover in a straight line and feeds the estimator. */
class Drive
{
public:
    explicit Drive(HeadingAlignmentEstimator & estimator)
    : estimator_(estimator) {}

    /**
     * @param enu_course_rad direction the antenna moves in ENU.
     * @param odom_yaw_rad   yaw reported by odometry while doing so.
     * @param vx_m_s         forward speed (negative = reversing).
     */
    void straight(
        double enu_course_rad, double odom_yaw_rad, double vx_m_s, double seconds,
        double yaw_rate_rad_s = 0.0)
    {
        const double speed = std::abs(vx_m_s);
        for (int step = 0; step < static_cast<int>(seconds * 10.0); ++step) {
            t_ += 0.1;
            east_m_ += 0.1 * speed * std::cos(enu_course_rad);
            north_m_ += 0.1 * speed * std::sin(enu_course_rad);

            OdometrySample odom;
            odom.stamp_s = t_;
            odom.yaw_rad = wrapAngle(odom_yaw_rad + yaw_rate_rad_s * (t_ - start_t_));
            odom.vx_m_s = vx_m_s;
            odom.yaw_rate_rad_s = yaw_rate_rad_s;
            estimator_.addOdometry(odom);

            if (step % 10 == 9) {  // 1 Hz GNSS
                estimator_.addFix(fixHere());
            }
        }
    }

private:
    GnssFix fixHere() const
    {
        GnssFix fix;
        fix.latitude_deg = kLat0 + north_m_ / kEarthRadiusM * 180.0 / kPi;
        fix.longitude_deg =
            kLon0 + east_m_ / (kEarthRadiusM * std::cos(kLat0 * kPi / 180.0)) * 180.0 / kPi;
        fix.status = FixStatus::Fix;
        fix.horizontal_std_m = 1.0;
        fix.stamp_s = t_;
        return fix;
    }

    HeadingAlignmentEstimator & estimator_;
    double t_{0.0};
    double start_t_{0.0};
    double east_m_{0.0};
    double north_m_{0.0};
};

}  // namespace

TEST(HeadingAlignmentEstimatorTest, AlignsWhileDrivingStraight)
{
    HeadingAlignmentEstimator estimator(HeadingAlignmentConfig{});
    Drive drive(estimator);

    // Odom says yaw 0 while the rover actually drives north-east (ENU 45 deg).
    drive.straight(kPi / 4.0, 0.0, 1.0, 15.0);

    ASSERT_TRUE(estimator.aligned());
    const AlignmentStatus status = estimator.status();
    EXPECT_EQ(status.accepted_segments, 3u);
    ASSERT_TRUE(status.yaw_offset_rad.has_value());
    EXPECT_NEAR(*status.yaw_offset_rad, kPi / 4.0, 0.01);
    ASSERT_TRUE(estimator.enuYaw(0.5).has_value());
    EXPECT_NEAR(*estimator.enuYaw(0.5), 0.5 + kPi / 4.0, 0.01);
}

TEST(HeadingAlignmentEstimatorTest, NotAlignedBeforeEnoughSegments)
{
    HeadingAlignmentEstimator estimator(HeadingAlignmentConfig{});
    Drive drive(estimator);

    drive.straight(0.0, 0.0, 1.0, 5.0);

    EXPECT_FALSE(estimator.aligned());
    EXPECT_EQ(estimator.status().state, AlignmentState::Collecting);
    EXPECT_FALSE(estimator.enuYaw(0.0).has_value());
}

TEST(HeadingAlignmentEstimatorTest, TurningIsRejected)
{
    HeadingAlignmentEstimator estimator(HeadingAlignmentConfig{});
    Drive drive(estimator);

    drive.straight(0.0, 0.0, 1.0, 30.0, 0.3);

    EXPECT_FALSE(estimator.aligned());
    EXPECT_EQ(estimator.status().accepted_segments, 0u);
}

TEST(HeadingAlignmentEstimatorTest, StandingStillIsRejected)
{
    HeadingAlignmentEstimator estimator(HeadingAlignmentConfig{});
    Drive drive(estimator);

    drive.straight(0.0, 0.0, 0.0, 30.0);

    EXPECT_EQ(estimator.status().state, AlignmentState::WaitingForMotion);
}

TEST(HeadingAlignmentEstimatorTest, ReversingAddsHalfTurn)
{
    HeadingAlignmentEstimator estimator(HeadingAlignmentConfig{});
    Drive drive(estimator);

    // Base points east in ENU (odom yaw 0.2), rover reverses → antenna moves west.
    drive.straight(kPi, 0.2, -1.0, 15.0);

    ASSERT_TRUE(estimator.aligned());
    EXPECT_NEAR(*estimator.status().yaw_offset_rad, -0.2, 0.01);
}

TEST(HeadingAlignmentEstimatorTest, OffsetAcrossWrapAround)
{
    HeadingAlignmentEstimator estimator(HeadingAlignmentConfig{});
    Drive drive(estimator);

    // Offset of ~180 deg: segments land on both sides of +-pi.
    drive.straight(-kPi / 2.0, kPi / 2.0 - 0.001, 1.0, 15.0);

    ASSERT_TRUE(estimator.aligned());
    EXPECT_NEAR(std::abs(*estimator.status().yaw_offset_rad), kPi, 0.01);
}

TEST(HeadingAlignmentEstimatorTest, NoFixIsIgnored)
{
    HeadingAlignmentConfig config;
    config.required_segments = 1;
    HeadingAlignmentEstimator estimator(config);

    OdometrySample odom;
    odom.stamp_s = 1.0;
    odom.vx_m_s = 1.0;
    estimator.addOdometry(odom);

    GnssFix fix;
    fix.status = FixStatus::NoFix;
    fix.stamp_s = 1.0;
    estimator.addFix(fix);

    EXPECT_EQ(estimator.status().state, AlignmentState::WaitingForMotion);
}

TEST(HeadingAlignmentEstimatorTest, ResetClearsAlignment)
{
    HeadingAlignmentEstimator estimator(HeadingAlignmentConfig{});
    Drive drive(estimator);
    drive.straight(0.0, 0.0, 1.0, 15.0);
    ASSERT_TRUE(estimator.aligned());

    estimator.reset();

    EXPECT_FALSE(estimator.aligned());
    EXPECT_EQ(estimator.status().accepted_segments, 0u);
    EXPECT_FALSE(estimator.status().yaw_offset_rad.has_value());
}

TEST(HeadingAlignmentEstimatorTest, RejectsInvalidConfig)
{
    HeadingAlignmentConfig config;
    config.required_segments = 0;
    EXPECT_THROW(HeadingAlignmentEstimator{config}, std::invalid_argument);

    HeadingAlignmentConfig negative;
    negative.min_segment_length_m = -1.0;
    EXPECT_THROW(HeadingAlignmentEstimator{negative}, std::invalid_argument);
}
