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

#include "rover_gps/domain/heading_alignment_estimator.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>

#include "rover_gps/domain/geo_math.hpp"

namespace rover_gps::domain
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double directionOf(double vx_m_s)
{
    return vx_m_s < 0.0 ? -1.0 : 1.0;
}
}  // namespace

HeadingAlignmentEstimator::HeadingAlignmentEstimator(HeadingAlignmentConfig config)
: config_(config)
{
    validate(config_);
}

void HeadingAlignmentEstimator::validate(const HeadingAlignmentConfig & config)
{
    if (!(config.min_segment_length_m > 0.0) || !(config.min_speed_m_s > 0.0) ||
        !(config.max_yaw_rate_rad_s > 0.0) || !(config.max_yaw_change_rad > 0.0) ||
        !(config.max_horizontal_std_m > 0.0) || !(config.max_fix_gap_s > 0.0) ||
        !(config.max_odometry_age_s > 0.0) || !(config.max_offset_std_rad > 0.0))
    {
        throw std::invalid_argument("Heading alignment limits must be positive.");
    }
    if (config.required_segments == 0) {
        throw std::invalid_argument("Heading alignment needs at least one segment.");
    }
}

void HeadingAlignmentEstimator::addOdometry(const OdometrySample & sample)
{
    last_odometry_ = sample;

    if (aligned() || !segment_) {
        return;
    }

    const bool same_direction = directionOf(sample.vx_m_s) == segment_->direction;
    const bool heading_kept =
        std::abs(wrapAngle(sample.yaw_rad - segment_->start_yaw_rad)) <= config_.max_yaw_change_rad;

    if (!motionIsStraight(sample) || !same_direction || !heading_kept) {
        abortSegment();
        return;
    }

    segment_->sum_sin += std::sin(sample.yaw_rad);
    segment_->sum_cos += std::cos(sample.yaw_rad);
}

void HeadingAlignmentEstimator::addFix(const GnssFix & fix)
{
    if (aligned()) {
        return;
    }

    const std::optional<GnssFix> previous_fix = last_fix_;
    last_fix_ = fix;

    const bool usable_fix = hasFix(fix.status) &&
        (std::isnan(fix.horizontal_std_m) || fix.horizontal_std_m <= config_.max_horizontal_std_m);
    const bool fresh_straight_odometry = last_odometry_ &&
        std::abs(fix.stamp_s - last_odometry_->stamp_s) <= config_.max_odometry_age_s &&
        motionIsStraight(*last_odometry_);

    if (!usable_fix || !fresh_straight_odometry) {
        abortSegment();
        return;
    }

    if (segment_ && previous_fix && fix.stamp_s - previous_fix->stamp_s > config_.max_fix_gap_s) {
        abortSegment();
    }

    if (segment_ && directionOf(last_odometry_->vx_m_s) != segment_->direction) {
        abortSegment();
    }

    if (!segment_) {
        startSegment(fix);
        return;
    }

    const EnuOffset displacement = enuOffset(
        segment_->start_fix.latitude_deg, segment_->start_fix.longitude_deg,
        fix.latitude_deg, fix.longitude_deg);
    if (std::hypot(displacement.east_m, displacement.north_m) < config_.min_segment_length_m) {
        return;
    }

    acceptSegment(fix);
    if (!aligned()) {
        // Keep driving straight → the next segment starts where this one ended.
        startSegment(fix);
    }
}

void HeadingAlignmentEstimator::reset()
{
    state_ = AlignmentState::WaitingForMotion;
    segment_.reset();
    last_fix_.reset();
    offsets_rad_.clear();
    yaw_offset_rad_ = 0.0;
    offset_std_rad_ = 0.0;
}

AlignmentStatus HeadingAlignmentEstimator::status() const
{
    AlignmentStatus status;
    status.state = state_;
    status.accepted_segments = offsets_rad_.size();
    if (!offsets_rad_.empty()) {
        status.yaw_offset_rad = yaw_offset_rad_;
        status.offset_std_rad = offset_std_rad_;
    }
    return status;
}

std::optional<double> HeadingAlignmentEstimator::enuYaw(double odom_yaw_rad) const
{
    if (!aligned()) {
        return std::nullopt;
    }
    return wrapAngle(odom_yaw_rad + yaw_offset_rad_);
}

bool HeadingAlignmentEstimator::motionIsStraight(const OdometrySample & sample) const
{
    return std::abs(sample.vx_m_s) >= config_.min_speed_m_s &&
           std::abs(sample.yaw_rate_rad_s) <= config_.max_yaw_rate_rad_s;
}

void HeadingAlignmentEstimator::startSegment(const GnssFix & fix)
{
    Segment segment;
    segment.start_fix = fix;
    segment.start_yaw_rad = last_odometry_->yaw_rad;
    segment.sum_sin = std::sin(last_odometry_->yaw_rad);
    segment.sum_cos = std::cos(last_odometry_->yaw_rad);
    segment.direction = directionOf(last_odometry_->vx_m_s);
    segment_ = segment;
    state_ = AlignmentState::Collecting;
}

void HeadingAlignmentEstimator::abortSegment()
{
    segment_.reset();
    if (offsets_rad_.empty()) {
        state_ = AlignmentState::WaitingForMotion;
    }
}

void HeadingAlignmentEstimator::acceptSegment(const GnssFix & end_fix)
{
    const EnuOffset displacement = enuOffset(
        segment_->start_fix.latitude_deg, segment_->start_fix.longitude_deg,
        end_fix.latitude_deg, end_fix.longitude_deg);

    const double course_rad = std::atan2(displacement.north_m, displacement.east_m);
    const double mean_odom_yaw_rad = std::atan2(segment_->sum_sin, segment_->sum_cos);
    // Reversing: the antenna moves opposite to where the base points.
    const double motion_yaw_rad =
        segment_->direction > 0.0 ? mean_odom_yaw_rad : mean_odom_yaw_rad + kPi;

    offsets_rad_.push_back(wrapAngle(course_rad - motion_yaw_rad));
    if (offsets_rad_.size() > config_.required_segments) {
        offsets_rad_.erase(offsets_rad_.begin());
    }
    segment_.reset();

    updateOffsetStats();
}

void HeadingAlignmentEstimator::updateOffsetStats()
{
    const CircularStats stats = circularStats(offsets_rad_);
    yaw_offset_rad_ = stats.mean_rad;
    offset_std_rad_ = stats.std_rad;

    if (offsets_rad_.size() >= config_.required_segments &&
        offset_std_rad_ <= config_.max_offset_std_rad)
    {
        state_ = AlignmentState::Aligned;
    } else {
        state_ = AlignmentState::Collecting;
    }
}

}  // namespace rover_gps::domain
