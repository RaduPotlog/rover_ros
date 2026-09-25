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

#ifndef ROVER_GPS_HEADING_DOMAIN_GEO_MATH_HPP_
#define ROVER_GPS_HEADING_DOMAIN_GEO_MATH_HPP_

#include <vector>

namespace rover_gps_heading::domain
{

/** @brief Local East-North offset [m]. */
struct EnuOffset
{
    double east_m{0.0};
    double north_m{0.0};
};

/**
 * @brief Equirectangular East/North offset of `to` relative to `from`.
 * @details Accurate to well under 0.1 % over the tens of metres a heading segment spans.
 */
EnuOffset enuOffset(
    double from_latitude_deg, double from_longitude_deg,
    double to_latitude_deg, double to_longitude_deg);

/** @brief Wraps an angle to [-pi, pi). */
double wrapAngle(double angle_rad);

/** @brief Yaw [rad] of a unit quaternion (ZYX convention), in [-pi, pi]. */
double yawFromQuaternion(double x, double y, double z, double w);

/** @brief Circular mean and circular standard deviation of a set of angles. */
struct CircularStats
{
    double mean_rad{0.0};
    double std_rad{0.0};
};

/** @pre `angles_rad` is not empty. */
CircularStats circularStats(const std::vector<double> & angles_rad);

}  // namespace rover_gps_heading::domain

#endif  // ROVER_GPS_HEADING_DOMAIN_GEO_MATH_HPP_
