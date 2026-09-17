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
//
// Field layout derived from nmea_navsat_driver (BSD-3-Clause, Copyright (c) 2013 Eric Perko):
// libnmea_navsat_driver/parser.py `parse_maps`.

#ifndef ROVER_GPS_DOMAIN_NMEA_NMEA_SENTENCE_HPP_
#define ROVER_GPS_DOMAIN_NMEA_NMEA_SENTENCE_HPP_

#include <limits>
#include <variant>

namespace rover_gps::domain::nmea
{

/** @brief Not-a-number, used for every field the receiver left empty. */
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/** @brief The sentence types this driver understands. */
enum class SentenceType
{
    Gga,
    Rmc,
    Vtg,
    Gst,
    Hdt,
};

/**
 * @brief Global positioning system fix data.
 * @note Latitude and longitude are already signed: the parser applies the N/S and E/W hemisphere
 *       characters, so consumers never see the raw direction fields.
 */
struct GgaData
{
    // GGA quality indicator: 0 invalid, 1 SPS, 2 DGPS, 4 RTK fixed, 5 RTK float, 9 WAAS.
    int fix_type{0};
    double latitude_deg{kNaN};
    double longitude_deg{kNaN};
    double altitude_m{kNaN};
    double mean_sea_level_m{kNaN};
    double hdop{kNaN};
    int num_satellites{0};
    // Seconds since the UNIX epoch, or NaN when the receiver sent no time.
    double utc_time_s{kNaN};
};

/** @brief Recommended minimum specific GNSS data. */
struct RmcData
{
    double utc_time_s{kNaN};
    bool fix_valid{false};
    double latitude_deg{kNaN};
    double longitude_deg{kNaN};
    double speed_m_s{kNaN};
    double true_course_rad{kNaN};
};

/** @brief Course over ground and ground speed. */
struct VtgData
{
    double true_course_rad{kNaN};
    double speed_m_s{kNaN};
};

/** @brief Pseudorange noise statistics: the receiver's own error estimate. */
struct GstData
{
    double utc_time_s{kNaN};
    double ranges_std_dev{kNaN};
    double semi_major_ellipse_std_dev{kNaN};
    double semi_minor_ellipse_std_dev{kNaN};
    double semi_major_orientation{kNaN};
    double lat_std_dev{kNaN};
    double lon_std_dev{kNaN};
    double alt_std_dev{kNaN};
};

/** @brief True heading. */
struct HdtData
{
    // Degrees clockwise from true north, as sent. Not converted to an ENU yaw.
    double heading_deg{kNaN};
};

/** @brief One successfully parsed sentence. */
using ParsedSentence = std::variant<GgaData, RmcData, VtgData, GstData, HdtData>;

/** @brief Discriminant of `sentence`, for switch-style dispatch and diagnostics. */
SentenceType typeOf(const ParsedSentence & sentence);

/** @brief Stable short name ("GGA", "RMC", ...) for logging. */
const char * sentenceTypeText(SentenceType type);

}  // namespace rover_gps::domain::nmea

#endif  // ROVER_GPS_DOMAIN_NMEA_NMEA_SENTENCE_HPP_
