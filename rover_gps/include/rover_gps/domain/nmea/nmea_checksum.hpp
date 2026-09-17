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
// libnmea_navsat_driver/checksum_utils.py.

#ifndef ROVER_GPS_DOMAIN_NMEA_NMEA_CHECKSUM_HPP_
#define ROVER_GPS_DOMAIN_NMEA_NMEA_CHECKSUM_HPP_

#include <string_view>

namespace rover_gps::domain::nmea
{

/**
 * @brief Verifies the trailing `*XX` checksum of an NMEA 0183 sentence.
 * @details XORs every character between the leading `$` and the `*`, and compares the result with
 *          the two hex digits that follow the `*` (case-insensitive).
 * @param sentence One sentence, without the trailing CR/LF.
 * @return false when there is no `*`, more than one `*`, fewer than two hex digits after it, or the
 *         checksum does not match.
 */
bool verifyChecksum(std::string_view sentence);

}  // namespace rover_gps::domain::nmea

#endif  // ROVER_GPS_DOMAIN_NMEA_NMEA_CHECKSUM_HPP_
