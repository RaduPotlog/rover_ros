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

#include "rover_gps_heading/domain/geo_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rover_gps_heading::domain
{

namespace
{
constexpr double kEarthRadiusM = 6378137.0;  // WGS-84 equatorial radius
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
}  // namespace

EnuOffset enuOffset(
    double from_latitude_deg, double from_longitude_deg,
    double to_latitude_deg, double to_longitude_deg)
{
    const double mean_latitude_rad = 0.5 * (from_latitude_deg + to_latitude_deg) * kDegToRad;
    const double d_latitude_rad = (to_latitude_deg - from_latitude_deg) * kDegToRad;
    // Wrap the longitude difference so a segment across the antimeridian stays short.
    const double d_longitude_rad = wrapAngle((to_longitude_deg - from_longitude_deg) * kDegToRad);

    return EnuOffset{
        kEarthRadiusM * d_longitude_rad * std::cos(mean_latitude_rad),
        kEarthRadiusM * d_latitude_rad};
}

double wrapAngle(double angle_rad)
{
    double wrapped = std::fmod(angle_rad + kPi, 2.0 * kPi);
    if (wrapped < 0.0) {
        wrapped += 2.0 * kPi;
    }
    return wrapped - kPi;
}

CircularStats circularStats(const std::vector<double> & angles_rad)
{
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for (const double angle : angles_rad) {
        sum_sin += std::sin(angle);
        sum_cos += std::cos(angle);
    }

    const double count = static_cast<double>(angles_rad.size());
    const double resultant = std::hypot(sum_sin, sum_cos) / count;

    CircularStats stats;
    stats.mean_rad = std::atan2(sum_sin, sum_cos);
    // Rounding can push the resultant slightly above 1 for identical angles.
    const double clamped = std::clamp(resultant, std::numeric_limits<double>::min(), 1.0);
    // max() also turns -0.0 (from log(1) = 0) into +0.0.
    stats.std_rad = std::sqrt(std::max(0.0, -2.0 * std::log(clamped)));
    return stats;
}

}  // namespace rover_gps_heading::domain
