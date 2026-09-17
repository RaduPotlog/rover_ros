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

#ifndef ROVER_RS16_LIDAR_DOMAIN_LIDAR_HEALTH_EVALUATOR_HPP_
#define ROVER_RS16_LIDAR_DOMAIN_LIDAR_HEALTH_EVALUATOR_HPP_

#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace rover_rs16_lidar::domain
{

enum class HealthLevel
{
    Ok,
    Warn,
    Error,
    Stale,
};

/** @brief One point cloud as far as the health check is concerned. */
struct CloudSample
{
    // Arrival time on the rover clock, not the message stamp: the driver may stamp with the
    // lidar's own clock (use_lidar_clock), which is not comparable to the rover's.
    double arrival_s{0.0};
    std::uint32_t point_count{0};
};

struct LidarHealthThresholds
{
    double expected_rate_hz{10.0};
    // Rate below expected_rate_hz * min_rate_ratio is reported as WARN.
    double min_rate_ratio{0.5};
    // No cloud for this long is an ERROR (lidar unplugged, or the rover is off its subnet).
    double cloud_timeout_s{2.0};
    // A cloud this sparse usually means a blocked or blinded sensor.
    std::uint32_t min_points_warn{1000};
};

struct LidarHealthReport
{
    HealthLevel level{HealthLevel::Stale};
    std::string message;
    // Everything below is only meaningful once a cloud has been received.
    bool has_data{false};
    double rate_hz{0.0};
    double age_s{0.0};
    std::uint32_t point_count{0};
    std::uint64_t cloud_count{0};
};

/** @brief Classifies the point-cloud stream: link alive, publish rate and cloud density. */
class LidarHealthEvaluator
{
public:
    explicit LidarHealthEvaluator(LidarHealthThresholds thresholds);

    /** @throws std::invalid_argument when a threshold is not positive. */
    static void validate(const LidarHealthThresholds & thresholds);

    void addCloud(const CloudSample & cloud);

    LidarHealthReport evaluate(double now_s) const;

private:
    double rateHz(double now_s) const;

    LidarHealthThresholds thresholds_;
    std::optional<CloudSample> last_cloud_;
    std::deque<double> arrival_times_s_;
    std::uint64_t cloud_count_{0};
};

}  // namespace rover_rs16_lidar::domain

#endif  // ROVER_RS16_LIDAR_DOMAIN_LIDAR_HEALTH_EVALUATOR_HPP_
