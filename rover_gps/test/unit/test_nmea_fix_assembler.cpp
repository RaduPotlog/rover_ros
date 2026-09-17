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

#include "rover_gps/domain/nmea/nmea_fix_assembler.hpp"

using namespace rover_gps::domain;       // NOLINT
using namespace rover_gps::domain::nmea;  // NOLINT

namespace
{

constexpr double kPi = 3.14159265358979323846;

GgaData makeGga(int fix_type, double hdop = 0.9)
{
    GgaData data;
    data.fix_type = fix_type;
    data.latitude_deg = 48.1173;
    data.longitude_deg = 11.516667;
    data.altitude_m = 545.4;
    data.mean_sea_level_m = 46.9;
    data.hdop = hdop;
    data.num_satellites = 8;
    data.utc_time_s = 1758112519.0;
    return data;
}

RmcData makeRmc(bool valid)
{
    RmcData data;
    data.utc_time_s = 1758112519.0;
    data.fix_valid = valid;
    data.latitude_deg = 48.1173;
    data.longitude_deg = 11.516667;
    data.speed_m_s = 10.0;
    data.true_course_rad = 0.0;  // due north
    return data;
}

}  // namespace

TEST(NmeaFixAssemblerTest, GgaProducesAFixAndATimeReference)
{
    NmeaFixAssembler assembler{FixAssemblerConfig{}};

    const AssemblerOutputs outputs = assembler.update(ParsedSentence{makeGga(1)});

    ASSERT_TRUE(outputs.fix.has_value());
    EXPECT_EQ(outputs.fix->status, FixStatus::Fix);
    EXPECT_EQ(outputs.fix->covariance_type, CovarianceType::Approximated);
    EXPECT_NEAR(outputs.fix->latitude_deg, 48.1173, 1e-9);
    EXPECT_NEAR(outputs.fix->longitude_deg, 11.516667, 1e-9);
    // GGA altitude is above the geoid; the geoid separation is added on.
    EXPECT_NEAR(outputs.fix->altitude_m, 545.4 + 46.9, 1e-9);

    ASSERT_TRUE(outputs.time_reference.has_value());
    EXPECT_NEAR(outputs.time_reference->utc_time_s, 1758112519.0, 1e-6);

    EXPECT_FALSE(outputs.velocity.has_value());
    EXPECT_FALSE(outputs.heading.has_value());
}

TEST(NmeaFixAssemblerTest, CovarianceScalesWithHdopAndTheQualityDefault)
{
    NmeaFixAssembler assembler{FixAssemblerConfig{}};

    // Quality 1 (SPS) defaults to a 4 m estimated position error.
    const AssemblerOutputs outputs = assembler.update(ParsedSentence{makeGga(1, 0.9)});
    ASSERT_TRUE(outputs.fix.has_value());

    EXPECT_NEAR(outputs.fix->position_covariance[0], (0.9 * 4.0) * (0.9 * 4.0), 1e-9);
    EXPECT_NEAR(outputs.fix->position_covariance[4], (0.9 * 4.0) * (0.9 * 4.0), 1e-9);
    // Altitude uses twice the default error and an extra factor of two, inherited from upstream.
    EXPECT_NEAR(outputs.fix->position_covariance[8], (2.0 * 0.9 * 8.0) * (2.0 * 0.9 * 8.0), 1e-9);

    // Off-diagonal terms stay zero.
    EXPECT_NEAR(outputs.fix->position_covariance[1], 0.0, 1e-12);
    EXPECT_NEAR(outputs.fix->position_covariance[5], 0.0, 1e-12);
}

TEST(NmeaFixAssemblerTest, MapsEveryGgaQualityToItsFixStatus)
{
    struct Case
    {
        int fix_type;
        FixStatus status;
        CovarianceType covariance_type;
    };

    const Case cases[] = {
        {0, FixStatus::NoFix, CovarianceType::Unknown},
        {1, FixStatus::Fix, CovarianceType::Approximated},
        {2, FixStatus::SbasFix, CovarianceType::Approximated},
        {4, FixStatus::GbasFix, CovarianceType::Approximated},
        {5, FixStatus::GbasFix, CovarianceType::Approximated},
        {9, FixStatus::GbasFix, CovarianceType::Approximated},
        // Anything outside the table falls back to the unknown row.
        {7, FixStatus::NoFix, CovarianceType::Unknown},
        {-1, FixStatus::NoFix, CovarianceType::Unknown},
    };

    for (const Case & test_case : cases) {
        NmeaFixAssembler assembler{FixAssemblerConfig{}};
        const AssemblerOutputs outputs = assembler.update(ParsedSentence{makeGga(test_case.fix_type)});

        ASSERT_TRUE(outputs.fix.has_value()) << "fix_type " << test_case.fix_type;
        EXPECT_EQ(outputs.fix->status, test_case.status) << "fix_type " << test_case.fix_type;
        EXPECT_EQ(outputs.fix->covariance_type, test_case.covariance_type)
            << "fix_type " << test_case.fix_type;
    }
}

TEST(NmeaFixAssemblerTest, VtgIsSuppressedUntilAValidGgaFix)
{
    NmeaFixAssembler assembler{FixAssemblerConfig{}};

    VtgData vtg;
    vtg.speed_m_s = 10.0;
    vtg.true_course_rad = 0.0;

    // No fix yet.
    EXPECT_FALSE(assembler.update(ParsedSentence{vtg}).velocity.has_value());

    // An invalid GGA still does not unlock it.
    assembler.update(ParsedSentence{makeGga(0)});
    EXPECT_FALSE(assembler.update(ParsedSentence{vtg}).velocity.has_value());

    // A valid one does.
    assembler.update(ParsedSentence{makeGga(1)});
    const AssemblerOutputs outputs = assembler.update(ParsedSentence{vtg});
    ASSERT_TRUE(outputs.velocity.has_value());

    // Course 0 is due north: all of the speed lands on the north axis.
    EXPECT_NEAR(outputs.velocity->east_m_s, 0.0, 1e-9);
    EXPECT_NEAR(outputs.velocity->north_m_s, 10.0, 1e-9);
}

TEST(NmeaFixAssemblerTest, VtgSplitsSpeedAcrossTheCourse)
{
    NmeaFixAssembler assembler{FixAssemblerConfig{}};
    assembler.update(ParsedSentence{makeGga(1)});

    VtgData vtg;
    vtg.speed_m_s = 10.0;
    vtg.true_course_rad = kPi / 2.0;  // due east

    const AssemblerOutputs outputs = assembler.update(ParsedSentence{vtg});
    ASSERT_TRUE(outputs.velocity.has_value());
    EXPECT_NEAR(outputs.velocity->east_m_s, 10.0, 1e-9);
    EXPECT_NEAR(outputs.velocity->north_m_s, 0.0, 1e-9);
}

TEST(NmeaFixAssemblerTest, GstOverridesTheQualityDefaults)
{
    NmeaFixAssembler assembler{FixAssemblerConfig{}};
    EXPECT_FALSE(assembler.usingReceiverEpe());

    GstData gst;
    gst.lat_std_dev = 3.0;
    gst.lon_std_dev = 4.0;
    gst.alt_std_dev = 5.0;

    // GST publishes nothing by itself; it only latches the receiver's error estimate.
    EXPECT_TRUE(assembler.update(ParsedSentence{gst}).empty());
    EXPECT_TRUE(assembler.usingReceiverEpe());

    const AssemblerOutputs outputs = assembler.update(ParsedSentence{makeGga(1, 0.9)});
    ASSERT_TRUE(outputs.fix.has_value());

    EXPECT_NEAR(outputs.fix->position_covariance[0], (0.9 * 4.0) * (0.9 * 4.0), 1e-9);
    EXPECT_NEAR(outputs.fix->position_covariance[4], (0.9 * 3.0) * (0.9 * 3.0), 1e-9);
    EXPECT_NEAR(outputs.fix->position_covariance[8], (2.0 * 0.9 * 5.0) * (2.0 * 0.9 * 5.0), 1e-9);
}

TEST(NmeaFixAssemblerTest, GgaAndVtgAreIgnoredWhenUseRmcIsSet)
{
    FixAssemblerConfig config;
    config.use_rmc = true;
    NmeaFixAssembler assembler{config};

    EXPECT_TRUE(assembler.update(ParsedSentence{makeGga(1)}).empty());

    VtgData vtg;
    vtg.speed_m_s = 10.0;
    vtg.true_course_rad = 0.0;
    EXPECT_TRUE(assembler.update(ParsedSentence{vtg}).empty());
}

TEST(NmeaFixAssemblerTest, RmcProducesAFixOnlyWhenUseRmcIsSet)
{
    FixAssemblerConfig config;
    config.use_rmc = true;
    NmeaFixAssembler assembler{config};

    const AssemblerOutputs outputs = assembler.update(ParsedSentence{makeRmc(true)});

    ASSERT_TRUE(outputs.fix.has_value());
    EXPECT_EQ(outputs.fix->status, FixStatus::Fix);
    EXPECT_EQ(outputs.fix->covariance_type, CovarianceType::Unknown);
    // RMC carries no altitude.
    EXPECT_TRUE(std::isnan(outputs.fix->altitude_m));
    ASSERT_TRUE(outputs.time_reference.has_value());
    ASSERT_TRUE(outputs.velocity.has_value());
}

TEST(NmeaFixAssemblerTest, RmcStillProducesVelocityWhenGgaIsTheFixSource)
{
    NmeaFixAssembler assembler{FixAssemblerConfig{}};

    const AssemblerOutputs outputs = assembler.update(ParsedSentence{makeRmc(true)});

    // No fix, because use_rmc is false - but the velocity comes through, since GGA has none.
    EXPECT_FALSE(outputs.fix.has_value());
    EXPECT_FALSE(outputs.time_reference.has_value());
    ASSERT_TRUE(outputs.velocity.has_value());
    EXPECT_NEAR(outputs.velocity->north_m_s, 10.0, 1e-9);
}

TEST(NmeaFixAssemblerTest, AnInvalidRmcReportsNoFixAndNoVelocity)
{
    FixAssemblerConfig config;
    config.use_rmc = true;
    NmeaFixAssembler assembler{config};

    const AssemblerOutputs outputs = assembler.update(ParsedSentence{makeRmc(false)});

    ASSERT_TRUE(outputs.fix.has_value());
    EXPECT_EQ(outputs.fix->status, FixStatus::NoFix);
    EXPECT_FALSE(outputs.velocity.has_value());
}

TEST(NmeaFixAssemblerTest, HdtConvertsDegreesToRadians)
{
    NmeaFixAssembler assembler{FixAssemblerConfig{}};

    HdtData hdt;
    hdt.heading_deg = 90.0;

    const AssemblerOutputs outputs = assembler.update(ParsedSentence{hdt});
    ASSERT_TRUE(outputs.heading.has_value());
    EXPECT_NEAR(outputs.heading->yaw_rad, kPi / 2.0, 1e-9);
}

TEST(NmeaFixAssemblerTest, HdtPublishesDueNorthButNotNaN)
{
    NmeaFixAssembler assembler{FixAssemblerConfig{}};

    // Upstream tested the heading for truthiness, which dropped a valid 0 degrees and let NaN
    // through. Both are handled the other way round here.
    HdtData north;
    north.heading_deg = 0.0;
    const AssemblerOutputs due_north = assembler.update(ParsedSentence{north});
    ASSERT_TRUE(due_north.heading.has_value());
    EXPECT_NEAR(due_north.heading->yaw_rad, 0.0, 1e-12);

    HdtData missing;  // heading_deg defaults to NaN
    EXPECT_FALSE(assembler.update(ParsedSentence{missing}).heading.has_value());
}

TEST(NmeaFixAssemblerTest, ResetDropsTheLatchedState)
{
    NmeaFixAssembler assembler{FixAssemblerConfig{}};

    GstData gst;
    gst.lat_std_dev = 3.0;
    gst.lon_std_dev = 4.0;
    gst.alt_std_dev = 5.0;
    assembler.update(ParsedSentence{gst});
    assembler.update(ParsedSentence{makeGga(1)});
    EXPECT_TRUE(assembler.hasValidFix());
    EXPECT_TRUE(assembler.usingReceiverEpe());

    assembler.reset();

    EXPECT_FALSE(assembler.hasValidFix());
    EXPECT_FALSE(assembler.usingReceiverEpe());

    // Back to the configured default rather than the receiver estimate.
    const AssemblerOutputs outputs = assembler.update(ParsedSentence{makeGga(1, 0.9)});
    ASSERT_TRUE(outputs.fix.has_value());
    EXPECT_NEAR(outputs.fix->position_covariance[0], (0.9 * 4.0) * (0.9 * 4.0), 1e-9);
}

TEST(NmeaFixAssemblerTest, HonoursConfiguredQualityDefaults)
{
    FixAssemblerConfig config;
    config.epe.quality4 = 0.02;
    NmeaFixAssembler assembler{config};

    const AssemblerOutputs outputs = assembler.update(ParsedSentence{makeGga(4, 1.0)});
    ASSERT_TRUE(outputs.fix.has_value());
    EXPECT_EQ(outputs.fix->status, FixStatus::GbasFix);
    EXPECT_NEAR(outputs.fix->position_covariance[0], 0.02 * 0.02, 1e-12);
}
