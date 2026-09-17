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
// Derived from nmea_navsat_driver (BSD-3-Clause, Copyright (c) 2013 Eric Perko):
// libnmea_navsat_driver/parser.py.

#ifndef ROVER_GPS_DOMAIN_NMEA_NMEA_PARSER_HPP_
#define ROVER_GPS_DOMAIN_NMEA_NMEA_PARSER_HPP_

#include <ctime>
#include <optional>
#include <string_view>

#include "rover_gps/domain/nmea/nmea_sentence.hpp"

namespace rover_gps::domain::nmea
{

/**
 * @brief Parses one NMEA 0183 sentence.
 * @details Accepts only the `$GP`, `$GN`, `$GL` and `$IN` talkers followed by a `*XX` checksum
 *          suffix, matching the upstream driver; `$HEHDT` and proprietary `$P...` sentences are
 *          rejected. The checksum itself is not verified here - call `verifyChecksum()` first.
 * @param sentence One sentence, already trimmed of CR/LF and surrounding whitespace.
 * @param now_utc  Current UTC time, used to date the time-of-day that NMEA sends without a date.
 *                 Passed in rather than read from the clock so this stays pure and testable.
 * @return The parsed sentence, or `std::nullopt` when the talker, the suffix or the sentence type
 *         is not one this driver handles.
 */
std::optional<ParsedSentence> parseSentence(std::string_view sentence, std::time_t now_utc);

// --- Field converters -------------------------------------------------------------------------
// Exposed so the conversion rules can be tested directly; `parseSentence` is their only caller.

/** @return The value, or NaN when `field` is empty or not a number. */
double safeFloat(std::string_view field);

/** @return The value, or 0 when `field` is empty or not an integer. */
int safeInt(std::string_view field);

/** @brief Converts NMEA `ddmm.mmmm` latitude to decimal degrees (unsigned). */
double convertLatitude(std::string_view field);

/** @brief Converts NMEA `dddmm.mmmm` longitude to decimal degrees (unsigned). */
double convertLongitude(std::string_view field);

/** @brief Knots to metres per second. */
double knotsToMps(std::string_view field);

/** @brief Degrees to radians. */
double degToRad(std::string_view field);

/**
 * @brief Converts an NMEA `HHMMSS[.sss]` time of day to seconds since the UNIX epoch.
 * @details NMEA sends no date in these sentences, so the time of day is grafted onto the UTC date
 *          of `now_utc`. Sub-second digits are ignored, as upstream does.
 * @return The epoch seconds, or NaN when the field is too short to hold HHMMSS.
 */
double utcTimeOfDay(std::string_view field, std::time_t now_utc);

}  // namespace rover_gps::domain::nmea

#endif  // ROVER_GPS_DOMAIN_NMEA_NMEA_PARSER_HPP_
