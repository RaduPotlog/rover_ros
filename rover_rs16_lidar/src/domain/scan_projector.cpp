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

#include "rover_rs16_lidar/domain/scan_projector.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rover_rs16_lidar::domain
{

ScanProjector::ScanProjector(ScanSettings settings) : settings_(std::move(settings))
{
    if (settings_.min_height >= settings_.max_height) {
        throw std::invalid_argument("ScanProjector: min_height must be below max_height.");
    }
    if (settings_.angle_min >= settings_.angle_max) {
        throw std::invalid_argument("ScanProjector: angle_min must be below angle_max.");
    }
    if (settings_.angle_increment <= 0.0) {
        throw std::invalid_argument("ScanProjector: angle_increment must be positive.");
    }
    if (settings_.range_min < 0.0 || settings_.range_min >= settings_.range_max) {
        throw std::invalid_argument("ScanProjector: range_min must be in [0, range_max).");
    }

    const double span = settings_.angle_max - settings_.angle_min;
    bin_count_ = static_cast<std::size_t>(std::ceil(span / settings_.angle_increment));
    if (bin_count_ == 0) {
        throw std::invalid_argument("ScanProjector: angle_increment is wider than the range.");
    }

    // A beam that saw nothing is either +inf or a range the costmap will reject as too far.
    empty_bin_value_ = settings_.use_inf
                           ? std::numeric_limits<float>::infinity()
                           : static_cast<float>(settings_.range_max + 1.0);
}

LaserScanFrame ScanProjector::project(const PointCloudFrame & cloud) const
{
    LaserScanFrame scan;
    scan.timestamp_s = cloud.timestamp_s;
    scan.angle_min = settings_.angle_min;
    scan.angle_max = settings_.angle_max;
    scan.angle_increment = settings_.angle_increment;
    // The cloud is not ordered by bearing here, so there is no meaningful per-beam offset.
    scan.time_increment = 0.0;
    scan.scan_time = settings_.scan_time;
    scan.range_min = settings_.range_min;
    scan.range_max = settings_.range_max;
    scan.ranges.assign(bin_count_, empty_bin_value_);

    for (const auto & point : cloud.points) {
        // Invalid returns arrive as NaN whenever dense_points is false.
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }
        if (point.z < settings_.min_height || point.z > settings_.max_height) {
            continue;
        }

        const double range = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
        if (range < settings_.range_min || range > settings_.range_max) {
            continue;
        }

        const double angle = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x));
        if (angle < settings_.angle_min || angle > settings_.angle_max) {
            continue;
        }

        auto index = static_cast<std::size_t>((angle - settings_.angle_min) /
                                              settings_.angle_increment);
        // A bearing of exactly angle_max lands one past the last bin. Folding it back in
        // rather than dropping it keeps a return from directly behind the rover (atan2
        // returns exactly +pi there) out of a half-degree blind spot.
        if (index >= bin_count_) {
            index = bin_count_ - 1;
        }

        // Closest return wins: an obstacle must never be hidden by something behind it.
        const auto candidate = static_cast<float>(range);
        if (candidate < scan.ranges[index]) {
            scan.ranges[index] = candidate;
        }
    }

    return scan;
}

}  // namespace rover_rs16_lidar::domain
