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

#include "rover_gps/domain/nmea/nmea_checksum.hpp"

namespace rover_gps::domain::nmea
{

namespace
{

/** @return 0-15 for a hex digit, -1 otherwise. */
int hexValue(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

}  // namespace

bool verifyChecksum(std::string_view sentence)
{
    const std::size_t star = sentence.find('*');
    if (star == std::string_view::npos) {
        return false;
    }

    // The Python original splits on '*' and requires exactly two parts, so a second '*' anywhere
    // rejects the sentence.
    if (sentence.find('*', star + 1) != std::string_view::npos) {
        return false;
    }

    const std::string_view transmitted = sentence.substr(star + 1);
    if (transmitted.size() != 2) {
        return false;
    }

    const int high = hexValue(transmitted[0]);
    const int low = hexValue(transmitted[1]);
    if (high < 0 || low < 0) {
        return false;
    }

    // Everything between the leading '$' and the '*'. A sentence with no '$' would checksum its
    // own first character away, so require the marker explicitly.
    if (sentence.empty() || sentence.front() != '$') {
        return false;
    }

    unsigned int checksum = 0;
    for (std::size_t i = 1; i < star; ++i) {
        checksum ^= static_cast<unsigned char>(sentence[i]);
    }

    return checksum == static_cast<unsigned int>(high * 16 + low);
}

}  // namespace rover_gps::domain::nmea
