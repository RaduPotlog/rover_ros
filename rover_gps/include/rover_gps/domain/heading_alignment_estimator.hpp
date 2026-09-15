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

#ifndef ROVER_GPS_DOMAIN_HEADING_ALIGNMENT_ESTIMATOR_HPP_
#define ROVER_GPS_DOMAIN_HEADING_ALIGNMENT_ESTIMATOR_HPP_

#include <cstddef>
#include <optional>
#include <vector>

#include "rover_gps/domain/gnss_fix.hpp"

namespace rover_gps::domain
{

struct HeadingAlignmentConfig
{
    // Straight displacement between two fixes that makes one course measurement [m].
    double min_segment_length_m{3.0};
    // |forward speed| required during the whole segment [m/s].
    double min_speed_m_s{0.3};
    // |yaw rate| allowed during the segment [rad/s].
    double max_yaw_rate_rad_s{0.1};
    // Yaw change allowed between segment start and any sample inside it [rad].
    double max_yaw_change_rad{0.1};
    // Fixes with a worse horizontal 1-sigma error are not used [m].
    double max_horizontal_std_m{5.0};
    // A longer gap between fixes restarts the segment [s].
    double max_fix_gap_s{2.5};
    // Odometry older than this, when a fix arrives, is not trusted [s].
    double max_odometry_age_s{0.5};
    // Accepted segments needed before the offset is declared aligned.
    std::size_t required_segments{3};
    // Circular std of the segment offsets that the alignment must reach [rad].
    double max_offset_std_rad{0.1};
};

enum class AlignmentState
{
    WaitingForMotion,
    Collecting,
    Aligned,
};

struct AlignmentStatus
{
    AlignmentState state{AlignmentState::WaitingForMotion};
    std::size_t accepted_segments{0};
    // Set once at least one segment was accepted.
    std::optional<double> yaw_offset_rad;
    std::optional<double> offset_std_rad;
};

/**
 * @brief Estimates the fixed yaw offset between the odom frame and ENU from GNSS course.
 * @details While the rover drives straight, the course between two fixes (atan2(dN, dE))
 *          is its ENU heading, and the odometry yaw over the same stretch is its odom heading;
 *          their difference is one offset measurement. The antenna lever arm is a pure
 *          translation while not turning, so it does not bias the course. Once
 *          `required_segments` measurements agree within `max_offset_std_rad`, the circular mean
 *          is latched until reset().
 */
class HeadingAlignmentEstimator
{
public:
    explicit HeadingAlignmentEstimator(HeadingAlignmentConfig config);

    /** @throws std::invalid_argument when the configuration is not usable. */
    static void validate(const HeadingAlignmentConfig & config);

    void addOdometry(const OdometrySample & sample);

    void addFix(const GnssFix & fix);

    void reset();

    bool aligned() const {return state_ == AlignmentState::Aligned;}

    AlignmentStatus status() const;

    /** @brief ENU yaw of the base for an odom-frame yaw; empty until aligned. */
    std::optional<double> enuYaw(double odom_yaw_rad) const;

private:
    struct Segment
    {
        GnssFix start_fix;
        double start_yaw_rad{0.0};
        double sum_sin{0.0};
        double sum_cos{0.0};
        double direction{1.0};  // +1 forward, -1 reversing
    };

    bool motionIsStraight(const OdometrySample & sample) const;
    void startSegment(const GnssFix & fix);
    void abortSegment();
    void acceptSegment(const GnssFix & end_fix);
    void updateOffsetStats();

    HeadingAlignmentConfig config_;
    AlignmentState state_{AlignmentState::WaitingForMotion};
    std::optional<OdometrySample> last_odometry_;
    std::optional<Segment> segment_;
    std::optional<GnssFix> last_fix_;
    // The most recent `required_segments` offset measurements.
    std::vector<double> offsets_rad_;
    double yaw_offset_rad_{0.0};
    double offset_std_rad_{0.0};
};

}  // namespace rover_gps::domain

#endif  // ROVER_GPS_DOMAIN_HEADING_ALIGNMENT_ESTIMATOR_HPP_
