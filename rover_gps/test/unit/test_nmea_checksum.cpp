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

#include "rover_gps/domain/nmea/nmea_checksum.hpp"

using namespace rover_gps::domain::nmea;  // NOLINT

TEST(NmeaChecksumTest, AcceptsAValidSentence)
{
    EXPECT_TRUE(
        verifyChecksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"));
}

TEST(NmeaChecksumTest, AcceptsLowercaseHexDigits)
{
    // 0x6A written as "6a" must still match.
    EXPECT_TRUE(
        verifyChecksum("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6a"));
}

TEST(NmeaChecksumTest, RejectsAWrongChecksum)
{
    EXPECT_FALSE(
        verifyChecksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48"));
}

TEST(NmeaChecksumTest, RejectsASentenceWithoutAChecksum)
{
    EXPECT_FALSE(verifyChecksum("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"));
}

TEST(NmeaChecksumTest, RejectsMoreThanOneStar)
{
    EXPECT_FALSE(verifyChecksum("$GPGGA,123519*4807.038*47"));
}

TEST(NmeaChecksumTest, RejectsAShortOrNonHexChecksum)
{
    EXPECT_FALSE(verifyChecksum("$GPHDT,123.456,T*3"));
    EXPECT_FALSE(verifyChecksum("$GPHDT,123.456,T*3Z"));
    EXPECT_FALSE(verifyChecksum("$GPHDT,123.456,T*321"));
}

TEST(NmeaChecksumTest, RejectsASentenceWithoutTheDollarMarker)
{
    // Without the '$' the first payload byte would be dropped from the XOR and a bogus sentence
    // could match by accident.
    EXPECT_FALSE(verifyChecksum("GPHDT,123.456,T*32"));
}

TEST(NmeaChecksumTest, RejectsEmptyInput)
{
    EXPECT_FALSE(verifyChecksum(""));
    EXPECT_FALSE(verifyChecksum("*47"));
}
