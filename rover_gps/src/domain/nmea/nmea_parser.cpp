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

#include "rover_gps/domain/nmea/nmea_parser.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

namespace rover_gps::domain::nmea
{

namespace
{

constexpr double kKnotsToMps = 0.514444444444;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/** @brief `substr` that clamps instead of throwing when `pos` is past the end. */
std::string_view clampedSubstr(std::string_view text, std::size_t pos, std::size_t count)
{
    if (pos >= text.size()) {
        return {};
    }
    return text.substr(pos, count);
}

bool isHexDigit(char c)
{
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

std::string_view trim(std::string_view text)
{
    const auto is_space = [](char c) {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f';
    };
    while (!text.empty() && is_space(text.front())) {
        text.remove_prefix(1);
    }
    while (!text.empty() && is_space(text.back())) {
        text.remove_suffix(1);
    }
    return text;
}

/** @brief Splits on ',' keeping empty fields, which NMEA uses for "value not available". */
std::vector<std::string_view> splitFields(std::string_view sentence)
{
    std::vector<std::string_view> fields;
    std::size_t start = 0;
    while (true) {
        const std::size_t comma = sentence.find(',', start);
        if (comma == std::string_view::npos) {
            fields.push_back(sentence.substr(start));
            break;
        }
        fields.push_back(sentence.substr(start, comma - start));
        start = comma + 1;
    }
    return fields;
}

/** @return The field at `index`, or an empty view when the sentence is too short. */
std::string_view field(const std::vector<std::string_view> & fields, std::size_t index)
{
    return index < fields.size() ? fields[index] : std::string_view{};
}

/** @brief Applies the N/S or E/W hemisphere character to an unsigned magnitude. */
double applyHemisphere(double magnitude, std::string_view direction, char negative)
{
    const std::string_view trimmed = trim(direction);
    if (!trimmed.empty() && trimmed.front() == negative) {
        return -magnitude;
    }
    return magnitude;
}

/** @brief "A" means the fix is valid; anything else (including "V") means it is not. */
bool convertStatusFlag(std::string_view field_text)
{
    return trim(field_text) == "A";
}

}  // namespace

double safeFloat(std::string_view field_text)
{
    const std::string_view trimmed = trim(field_text);
    if (trimmed.empty()) {
        return kNaN;
    }

    const std::string text(trimmed);
    char * end = nullptr;
    const double value = std::strtod(text.c_str(), &end);

    // Reject partial matches such as "1.0M" the way Python's float() does.
    if (end != text.c_str() + text.size()) {
        return kNaN;
    }
    return value;
}

int safeInt(std::string_view field_text)
{
    const std::string_view trimmed = trim(field_text);
    if (trimmed.empty()) {
        return 0;
    }

    const std::string text(trimmed);
    char * end = nullptr;
    const long value = std::strtol(text.c_str(), &end, 10);

    // Python's int() rejects "1.5" and "8x" outright; so do we.
    if (end != text.c_str() + text.size()) {
        return 0;
    }
    return static_cast<int>(value);
}

double convertLatitude(std::string_view field_text)
{
    // ddmm.mmmm: two degree digits, the rest is minutes.
    return safeFloat(clampedSubstr(field_text, 0, 2)) +
           safeFloat(clampedSubstr(field_text, 2, std::string_view::npos)) / 60.0;
}

double convertLongitude(std::string_view field_text)
{
    // dddmm.mmmm: three degree digits, the rest is minutes.
    return safeFloat(clampedSubstr(field_text, 0, 3)) +
           safeFloat(clampedSubstr(field_text, 3, std::string_view::npos)) / 60.0;
}

double knotsToMps(std::string_view field_text)
{
    return safeFloat(field_text) * kKnotsToMps;
}

double degToRad(std::string_view field_text)
{
    return safeFloat(field_text) * kDegToRad;
}

double utcTimeOfDay(std::string_view field_text, std::time_t now_utc)
{
    const std::string_view trimmed = trim(field_text);
    if (trimmed.size() < 6) {
        return kNaN;
    }

    const auto two_digits = [trimmed](std::size_t pos) -> int {
        const char high = trimmed[pos];
        const char low = trimmed[pos + 1];
        if (high < '0' || high > '9' || low < '0' || low > '9') {
            return -1;
        }
        return (high - '0') * 10 + (low - '0');
    };

    const int hours = two_digits(0);
    const int minutes = two_digits(2);
    const int seconds = two_digits(4);
    if (hours < 0 || minutes < 0 || seconds < 0) {
        return kNaN;
    }

    // NMEA sends no date here, so graft the time of day onto today's UTC date.
    std::tm utc{};
    if (gmtime_r(&now_utc, &utc) == nullptr) {
        return kNaN;
    }
    utc.tm_hour = hours;
    utc.tm_min = minutes;
    utc.tm_sec = seconds;
    utc.tm_isdst = 0;

    const std::time_t epoch = timegm(&utc);
    if (epoch == static_cast<std::time_t>(-1)) {
        return kNaN;
    }
    return static_cast<double>(epoch);
}

std::optional<ParsedSentence> parseSentence(std::string_view sentence, std::time_t now_utc)
{
    // Talker whitelist, matching the upstream regex: only GP, GN, GL and IN.
    static constexpr std::array<std::string_view, 4> kTalkers{"$GP", "$GN", "$GL", "$IN"};

    if (sentence.size() < 4) {
        return std::nullopt;
    }

    const std::string_view prefix = sentence.substr(0, 3);
    bool talker_ok = false;
    for (const std::string_view talker : kTalkers) {
        if (prefix == talker) {
            talker_ok = true;
            break;
        }
    }
    if (!talker_ok) {
        return std::nullopt;
    }

    // ... and the sentence must end in '*' followed by exactly two hex digits.
    if (sentence.size() < 3 || sentence[sentence.size() - 3] != '*' ||
        !isHexDigit(sentence[sentence.size() - 2]) || !isHexDigit(sentence.back()))
    {
        return std::nullopt;
    }

    // Drop the "*XX" suffix before splitting. Upstream leaves it attached to the final field,
    // which is harmless for most sentences but silently turns GST's alt_std_dev - the last field -
    // into NaN, discarding the receiver's altitude error estimate on every GST.
    const std::vector<std::string_view> fields =
        splitFields(sentence.substr(0, sentence.size() - 3));

    // fields[0] is "$GPGGA"; drop the '$' and the two-letter talker.
    const std::string_view type_text = clampedSubstr(fields.front(), 3, std::string_view::npos);

    // Upstream indexes the field list blindly and dies on a truncated sentence. We bounds-check
    // instead: a short sentence yields empty fields, which the converters turn into NaN / 0.
    if (type_text == "GGA") {
        GgaData data;
        data.fix_type = safeInt(field(fields, 6));
        data.latitude_deg =
            applyHemisphere(convertLatitude(field(fields, 2)), field(fields, 3), 'S');
        data.longitude_deg =
            applyHemisphere(convertLongitude(field(fields, 4)), field(fields, 5), 'W');
        data.altitude_m = safeFloat(field(fields, 9));
        data.mean_sea_level_m = safeFloat(field(fields, 11));
        data.hdop = safeFloat(field(fields, 8));
        data.num_satellites = safeInt(field(fields, 7));
        data.utc_time_s = utcTimeOfDay(field(fields, 1), now_utc);
        return ParsedSentence{data};
    }

    if (type_text == "RMC") {
        RmcData data;
        data.utc_time_s = utcTimeOfDay(field(fields, 1), now_utc);
        data.fix_valid = convertStatusFlag(field(fields, 2));
        data.latitude_deg =
            applyHemisphere(convertLatitude(field(fields, 3)), field(fields, 4), 'S');
        data.longitude_deg =
            applyHemisphere(convertLongitude(field(fields, 5)), field(fields, 6), 'W');
        data.speed_m_s = knotsToMps(field(fields, 7));
        data.true_course_rad = degToRad(field(fields, 8));
        return ParsedSentence{data};
    }

    if (type_text == "VTG") {
        VtgData data;
        data.true_course_rad = degToRad(field(fields, 1));
        data.speed_m_s = knotsToMps(field(fields, 5));
        return ParsedSentence{data};
    }

    if (type_text == "GST") {
        GstData data;
        data.utc_time_s = utcTimeOfDay(field(fields, 1), now_utc);
        data.ranges_std_dev = safeFloat(field(fields, 2));
        data.semi_major_ellipse_std_dev = safeFloat(field(fields, 3));
        data.semi_minor_ellipse_std_dev = safeFloat(field(fields, 4));
        data.semi_major_orientation = safeFloat(field(fields, 5));
        data.lat_std_dev = safeFloat(field(fields, 6));
        data.lon_std_dev = safeFloat(field(fields, 7));
        data.alt_std_dev = safeFloat(field(fields, 8));
        return ParsedSentence{data};
    }

    if (type_text == "HDT") {
        HdtData data;
        data.heading_deg = safeFloat(field(fields, 1));
        return ParsedSentence{data};
    }

    return std::nullopt;
}

}  // namespace rover_gps::domain::nmea
