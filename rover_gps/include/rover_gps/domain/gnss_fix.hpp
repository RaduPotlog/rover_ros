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

#ifndef ROVER_GPS_DOMAIN_GNSS_FIX_HPP_
#define ROVER_GPS_DOMAIN_GNSS_FIX_HPP_

#include <cmath>
#include <limits>

namespace rover_gps::domain
{

/** @brief Fix quality, in the order of sensor_msgs/NavSatStatus. */
enum class FixStatus
{
    NoFix,
    Fix,
    SbasFix,
    GbasFix,
};

inline bool hasFix(FixStatus status)
{
    return status != FixStatus::NoFix;
}

/** @brief One GNSS position sample. */
struct GnssFix
{
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
    FixStatus status{FixStatus::NoFix};
    // Horizontal 1-sigma position error [m]; NaN when the receiver gives no estimate.
    double horizontal_std_m{std::numeric_limits<double>::quiet_NaN()};
    // Time the fix was received [s], on the same clock as OdometrySample::stamp_s.
    double stamp_s{0.0};
};

/** @brief Planar odometry sample of the base in the odom frame. */
struct OdometrySample
{
    double stamp_s{0.0};
    double yaw_rad{0.0};
    double vx_m_s{0.0};         // body-frame forward speed, negative while reversing
    double yaw_rate_rad_s{0.0};
};

}  // namespace rover_gps::domain

#endif  // ROVER_GPS_DOMAIN_GNSS_FIX_HPP_
