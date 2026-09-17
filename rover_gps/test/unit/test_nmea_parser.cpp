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

#include <gtest/gtest.h>

#include <cmath>
#include <ctime>

#include "rover_gps/domain/nmea/nmea_parser.hpp"

using namespace rover_gps::domain::nmea;  // NOLINT

namespace
{

// 2025-09-17T00:00:00Z. Fixed so the time-of-day grafting is deterministic.
constexpr std::time_t kNowUtc = 1758067200;

// 12:35:19 on that date.
constexpr double kExpectedUtc = 1758112519.0;

constexpr double kPi = 3.14159265358979323846;

}  // namespace

TEST(NmeaParserTest, ParsesGga)
{
    const auto parsed = parseSentence(
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47", kNowUtc);
    ASSERT_TRUE(parsed.has_value());
    ASSERT_EQ(typeOf(*parsed), SentenceType::Gga);

    const auto & gga = std::get<GgaData>(*parsed);
    EXPECT_EQ(gga.fix_type, 1);
    EXPECT_NEAR(gga.latitude_deg, 48.1173, 1e-4);      // 48 deg + 7.038 min
    EXPECT_NEAR(gga.longitude_deg, 11.516667, 1e-6);   // 11 deg + 31.000 min
    EXPECT_NEAR(gga.altitude_m, 545.4, 1e-6);
    EXPECT_NEAR(gga.mean_sea_level_m, 46.9, 1e-6);
    EXPECT_NEAR(gga.hdop, 0.9, 1e-9);
    EXPECT_EQ(gga.num_satellites, 8);
    EXPECT_NEAR(gga.utc_time_s, kExpectedUtc, 1e-6);
}

TEST(NmeaParserTest, AppliesSouthAndWestHemispheres)
{
    const auto parsed = parseSentence(
        "$GPGGA,123519,4807.038,S,01131.000,W,2,08,1.0,100.0,M,10.0,M,,*48", kNowUtc);
    ASSERT_TRUE(parsed.has_value());

    const auto & gga = std::get<GgaData>(*parsed);
    EXPECT_NEAR(gga.latitude_deg, -48.1173, 1e-4);
    EXPECT_NEAR(gga.longitude_deg, -11.516667, 1e-6);
}

TEST(NmeaParserTest, ParsesRmc)
{
    const auto parsed = parseSentence(
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A", kNowUtc);
    ASSERT_TRUE(parsed.has_value());
    ASSERT_EQ(typeOf(*parsed), SentenceType::Rmc);

    const auto & rmc = std::get<RmcData>(*parsed);
    EXPECT_TRUE(rmc.fix_valid);
    EXPECT_NEAR(rmc.latitude_deg, 48.1173, 1e-4);
    EXPECT_NEAR(rmc.longitude_deg, 11.516667, 1e-6);
    EXPECT_NEAR(rmc.speed_m_s, 22.4 * 0.514444444444, 1e-6);
    EXPECT_NEAR(rmc.true_course_rad, 84.4 * kPi / 180.0, 1e-9);
    EXPECT_NEAR(rmc.utc_time_s, kExpectedUtc, 1e-6);
}

TEST(NmeaParserTest, TreatsAnyStatusOtherThanAAsInvalid)
{
    const auto parsed = parseSentence(
        "$GPRMC,123519,V,4807.038,S,01131.000,W,022.4,084.4,230394,003.1,W*72", kNowUtc);
    ASSERT_TRUE(parsed.has_value());
    EXPECT_FALSE(std::get<RmcData>(*parsed).fix_valid);
}

TEST(NmeaParserTest, ParsesVtg)
{
    const auto parsed = parseSentence("$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48", kNowUtc);
    ASSERT_TRUE(parsed.has_value());
    ASSERT_EQ(typeOf(*parsed), SentenceType::Vtg);

    const auto & vtg = std::get<VtgData>(*parsed);
    EXPECT_NEAR(vtg.true_course_rad, 54.7 * kPi / 180.0, 1e-9);
    EXPECT_NEAR(vtg.speed_m_s, 5.5 * 0.514444444444, 1e-9);
}

TEST(NmeaParserTest, ParsesGst)
{
    const auto parsed = parseSentence("$GPGST,123519,1.2,2.5,1.8,15.0,3.0,4.0,5.0*4F", kNowUtc);
    ASSERT_TRUE(parsed.has_value());
    ASSERT_EQ(typeOf(*parsed), SentenceType::Gst);

    const auto & gst = std::get<GstData>(*parsed);
    EXPECT_NEAR(gst.ranges_std_dev, 1.2, 1e-9);
    EXPECT_NEAR(gst.semi_major_ellipse_std_dev, 2.5, 1e-9);
    EXPECT_NEAR(gst.semi_minor_ellipse_std_dev, 1.8, 1e-9);
    EXPECT_NEAR(gst.semi_major_orientation, 15.0, 1e-9);
    EXPECT_NEAR(gst.lat_std_dev, 3.0, 1e-9);
    EXPECT_NEAR(gst.lon_std_dev, 4.0, 1e-9);
    // alt_std_dev is the last field, so it still carries the "*4F" checksum in the raw sentence.
    // Upstream parsed that as NaN and threw the receiver's altitude estimate away; the suffix is
    // stripped before splitting here, so the value survives.
    EXPECT_NEAR(gst.alt_std_dev, 5.0, 1e-9);
}

TEST(NmeaParserTest, TheChecksumSuffixDoesNotLeakIntoTheLastField)
{
    // A one-field sentence where the value and the checksum share a field.
    const auto parsed = parseSentence("$GPHDT,123.456,T*32", kNowUtc);
    ASSERT_TRUE(parsed.has_value());
    EXPECT_NEAR(std::get<HdtData>(*parsed).heading_deg, 123.456, 1e-9);

    const auto gst = parseSentence("$GPGST,123519,1.2,2.5,1.8,15.0,3.0,4.0,5.0*4F", kNowUtc);
    ASSERT_TRUE(gst.has_value());
    EXPECT_FALSE(std::isnan(std::get<GstData>(*gst).alt_std_dev));
}

TEST(NmeaParserTest, ParsesHdt)
{
    const auto parsed = parseSentence("$GPHDT,123.456,T*32", kNowUtc);
    ASSERT_TRUE(parsed.has_value());
    ASSERT_EQ(typeOf(*parsed), SentenceType::Hdt);
    EXPECT_NEAR(std::get<HdtData>(*parsed).heading_deg, 123.456, 1e-9);
}

TEST(NmeaParserTest, AcceptsEveryWhitelistedTalker)
{
    EXPECT_TRUE(
        parseSentence("$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59", kNowUtc)
            .has_value());
}

TEST(NmeaParserTest, RejectsTalkersOutsideTheWhitelist)
{
    // Matching upstream: a Hemisphere-talker heading and proprietary sentences are dropped.
    EXPECT_FALSE(parseSentence("$HEHDT,123.4,T*2B", kNowUtc).has_value());
    EXPECT_FALSE(parseSentence("$PUBXFOO,1,2*5A", kNowUtc).has_value());
}

TEST(NmeaParserTest, RejectsAnUnhandledSentenceType)
{
    EXPECT_FALSE(parseSentence("$GPZDA,123519,23,03,1994,00,00*42", kNowUtc).has_value());
}

TEST(NmeaParserTest, RejectsASentenceWithoutTheChecksumSuffix)
{
    // The suffix must be '*' plus exactly two hex digits, anchored at the end.
    EXPECT_FALSE(parseSentence("$GPHDT,123.456,T", kNowUtc).has_value());
}

TEST(NmeaParserTest, RejectsASentenceWithATrailingCarriageReturn)
{
    // The node trims each line before parsing precisely because this does not parse. Upstream did
    // not, which silently dropped every sentence but the last in a multi-sentence datagram.
    EXPECT_FALSE(
        parseSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r",
                      kNowUtc)
            .has_value());
}

TEST(NmeaParserTest, DoesNotReadPastTheEndOfATruncatedSentence)
{
    // Upstream indexes blindly and raises IndexError here, which killed its receive loop.
    const auto parsed = parseSentence("$GPGGA,123519,4807.038,N*27", kNowUtc);
    ASSERT_TRUE(parsed.has_value());

    const auto & gga = std::get<GgaData>(*parsed);
    EXPECT_EQ(gga.fix_type, 0);
    EXPECT_TRUE(std::isnan(gga.hdop));
    EXPECT_TRUE(std::isnan(gga.altitude_m));
}

TEST(NmeaParserTest, EmptyNumericFieldsBecomeNaN)
{
    const auto parsed = parseSentence("$GPHDT,,T*1B", kNowUtc);
    ASSERT_TRUE(parsed.has_value());
    EXPECT_TRUE(std::isnan(std::get<HdtData>(*parsed).heading_deg));
}

TEST(SafeConvertersTest, SafeFloatRejectsPartialNumbers)
{
    EXPECT_NEAR(safeFloat("545.4"), 545.4, 1e-9);
    EXPECT_TRUE(std::isnan(safeFloat("")));
    EXPECT_TRUE(std::isnan(safeFloat("M")));
    EXPECT_TRUE(std::isnan(safeFloat("1.0M")));
}

TEST(SafeConvertersTest, SafeIntFallsBackToZero)
{
    EXPECT_EQ(safeInt("08"), 8);
    EXPECT_EQ(safeInt(""), 0);
    EXPECT_EQ(safeInt("1.5"), 0);
    EXPECT_EQ(safeInt("8x"), 0);
}

TEST(SafeConvertersTest, ConvertsDegreesAndMinutes)
{
    EXPECT_NEAR(convertLatitude("4807.038"), 48.1173, 1e-4);
    EXPECT_NEAR(convertLongitude("01131.000"), 11.516667, 1e-6);
    EXPECT_TRUE(std::isnan(convertLatitude("")));
    EXPECT_TRUE(std::isnan(convertLongitude("")));
}

TEST(SafeConvertersTest, UtcTimeOfDayNeedsSixDigits)
{
    EXPECT_NEAR(utcTimeOfDay("123519", kNowUtc), kExpectedUtc, 1e-6);
    EXPECT_NEAR(utcTimeOfDay("123519.50", kNowUtc), kExpectedUtc, 1e-6);  // sub-seconds ignored
    EXPECT_TRUE(std::isnan(utcTimeOfDay("1235", kNowUtc)));
    EXPECT_TRUE(std::isnan(utcTimeOfDay("", kNowUtc)));
    EXPECT_TRUE(std::isnan(utcTimeOfDay("aabbcc", kNowUtc)));
}
