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

#include "rover_lidar/domain/lidar_health_evaluator.hpp"

#include <algorithm>
#include <stdexcept>

namespace rover_lidar::domain
{

namespace
{
// Rate is averaged over this many expected periods, and never less than this many seconds.
constexpr double kRateWindowPeriods = 10.0;
constexpr double kMinRateWindowS = 1.0;
}  // namespace

LidarHealthEvaluator::LidarHealthEvaluator(LidarHealthThresholds thresholds)
: thresholds_(thresholds)
{
    validate(thresholds_);
}

void LidarHealthEvaluator::validate(const LidarHealthThresholds & thresholds)
{
    if (!(thresholds.expected_rate_hz > 0.0) || !(thresholds.min_rate_ratio > 0.0) ||
        !(thresholds.cloud_timeout_s > 0.0))
    {
        throw std::invalid_argument("Lidar health thresholds must be positive.");
    }
}

void LidarHealthEvaluator::addCloud(const CloudSample & cloud)
{
    last_cloud_ = cloud;
    ++cloud_count_;

    arrival_times_s_.push_back(cloud.arrival_s);
    const double window_s =
        std::max(kMinRateWindowS, kRateWindowPeriods / thresholds_.expected_rate_hz);
    while (arrival_times_s_.size() > 1 && cloud.arrival_s - arrival_times_s_.front() > window_s) {
        arrival_times_s_.pop_front();
    }
}

double LidarHealthEvaluator::rateHz(double now_s) const
{
    if (arrival_times_s_.size() < 2) {
        return 0.0;
    }
    // Measured up to `now`, so a stream that stops decays instead of freezing at its old rate.
    const double span_s = std::max(now_s, arrival_times_s_.back()) - arrival_times_s_.front();
    if (span_s <= 0.0) {
        return 0.0;
    }
    return static_cast<double>(arrival_times_s_.size() - 1) / span_s;
}

LidarHealthReport LidarHealthEvaluator::evaluate(double now_s) const
{
    LidarHealthReport report;
    report.cloud_count = cloud_count_;

    if (!last_cloud_) {
        report.level = HealthLevel::Stale;
        report.message = "No lidar data yet.";
        return report;
    }

    report.has_data = true;
    report.age_s = std::max(0.0, now_s - last_cloud_->arrival_s);
    report.rate_hz = rateHz(now_s);
    report.point_count = last_cloud_->point_count;

    if (report.age_s > thresholds_.cloud_timeout_s) {
        report.level = HealthLevel::Error;
        report.message = "Lidar data timeout.";
    } else if (report.point_count < thresholds_.min_points_warn) {
        report.level = HealthLevel::Warn;
        report.message = "Lidar cloud too sparse.";
    } else if (report.rate_hz < thresholds_.expected_rate_hz * thresholds_.min_rate_ratio) {
        // Only judged once the window holds two clouds; a single cloud has no rate yet.
        report.level = arrival_times_s_.size() < 2 ? HealthLevel::Ok : HealthLevel::Warn;
        report.message = arrival_times_s_.size() < 2 ? "Lidar streaming." : "Lidar rate too low.";
    } else {
        report.level = HealthLevel::Ok;
        report.message = "Lidar streaming.";
    }

    return report;
}

}  // namespace rover_lidar::domain
