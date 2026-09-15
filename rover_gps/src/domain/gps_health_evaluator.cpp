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

#include "rover_gps/domain/gps_health_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace rover_gps::domain
{

namespace
{
// Rate is averaged over this many expected periods, and never less than this many seconds.
constexpr double kRateWindowPeriods = 5.0;
constexpr double kMinRateWindowS = 5.0;
}  // namespace

GpsHealthEvaluator::GpsHealthEvaluator(GpsHealthThresholds thresholds)
: thresholds_(thresholds)
{
    validate(thresholds_);
}

void GpsHealthEvaluator::validate(const GpsHealthThresholds & thresholds)
{
    if (!(thresholds.expected_rate_hz > 0.0) || !(thresholds.min_rate_ratio > 0.0) ||
        !(thresholds.fix_timeout_s > 0.0) || !(thresholds.warn_horizontal_std_m > 0.0) ||
        !(thresholds.error_horizontal_std_m > 0.0))
    {
        throw std::invalid_argument("GPS health thresholds must be positive.");
    }
    if (thresholds.warn_horizontal_std_m > thresholds.error_horizontal_std_m) {
        throw std::invalid_argument(
            "warn_horizontal_std_m must not be greater than error_horizontal_std_m.");
    }
}

void GpsHealthEvaluator::addFix(const GnssFix & fix)
{
    last_fix_ = fix;
    ++fix_count_;

    arrival_times_s_.push_back(fix.stamp_s);
    const double window_s =
        std::max(kMinRateWindowS, kRateWindowPeriods / thresholds_.expected_rate_hz);
    while (arrival_times_s_.size() > 1 && fix.stamp_s - arrival_times_s_.front() > window_s) {
        arrival_times_s_.pop_front();
    }
}

double GpsHealthEvaluator::rateHz(double now_s) const
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

GpsHealthReport GpsHealthEvaluator::evaluate(double now_s) const
{
    GpsHealthReport report;
    report.fix_count = fix_count_;

    if (!last_fix_) {
        report.level = HealthLevel::Stale;
        report.message = "No GPS data yet.";
        return report;
    }

    report.has_data = true;
    report.last_fix = *last_fix_;
    report.age_s = std::max(0.0, now_s - last_fix_->stamp_s);
    report.rate_hz = rateHz(now_s);

    const double std_m = last_fix_->horizontal_std_m;

    if (report.age_s > thresholds_.fix_timeout_s) {
        report.level = HealthLevel::Error;
        report.message = "GPS data timeout.";
    } else if (!hasFix(last_fix_->status)) {
        report.level = HealthLevel::Warn;
        report.message = "No GNSS fix.";
    } else if (!std::isnan(std_m) && std_m > thresholds_.error_horizontal_std_m) {
        report.level = HealthLevel::Error;
        report.message = "GNSS accuracy too low.";
    } else if (!std::isnan(std_m) && std_m > thresholds_.warn_horizontal_std_m) {
        report.level = HealthLevel::Warn;
        report.message = "GNSS accuracy degraded.";
    } else if (report.rate_hz < thresholds_.expected_rate_hz * thresholds_.min_rate_ratio) {
        // Only judged once the window holds two fixes; a single fix has no rate yet.
        report.level = arrival_times_s_.size() < 2 ? HealthLevel::Ok : HealthLevel::Warn;
        report.message = arrival_times_s_.size() < 2 ? "GNSS fix." : "GPS rate too low.";
    } else {
        report.level = HealthLevel::Ok;
        report.message = "GNSS fix.";
    }

    return report;
}

}  // namespace rover_gps::domain
