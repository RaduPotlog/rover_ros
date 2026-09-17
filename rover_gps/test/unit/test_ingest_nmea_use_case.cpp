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

#include <memory>
#include <vector>

#include "rover_gps/application/ingest_nmea_use_case.hpp"

using namespace rover_gps;              // NOLINT
using namespace rover_gps::application;  // NOLINT

namespace
{

constexpr std::time_t kNowUtc = 1758067200;  // 2025-09-17T00:00:00Z

/** @brief Records everything the use case pushes out. */
class RecordingOutput : public domain::NmeaOutputPort
{
public:
    void publishFix(const domain::nmea::FixOutput & fix, double stamp_s) override
    {
        fixes.push_back(fix);
        stamps.push_back(stamp_s);
    }

    void publishVelocity(const domain::nmea::VelocityOutput & velocity, double) override
    {
        velocities.push_back(velocity);
    }

    void publishHeading(const domain::nmea::HeadingOutput & heading, double) override
    {
        headings.push_back(heading);
    }

    void publishTimeReference(const domain::nmea::TimeRefOutput & time_ref, double) override
    {
        time_references.push_back(time_ref);
    }

    std::vector<domain::nmea::FixOutput> fixes;
    std::vector<domain::nmea::VelocityOutput> velocities;
    std::vector<domain::nmea::HeadingOutput> headings;
    std::vector<domain::nmea::TimeRefOutput> time_references;
    std::vector<double> stamps;
};

}  // namespace

class IngestNmeaUseCaseTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        output_ = std::make_shared<RecordingOutput>();
        use_case_ = std::make_unique<IngestNmeaUseCase>(
            domain::nmea::FixAssemblerConfig{}, output_);
    }

    std::shared_ptr<RecordingOutput> output_;
    std::unique_ptr<IngestNmeaUseCase> use_case_;
};

TEST_F(IngestNmeaUseCaseTest, PublishesAFixFromAValidGga)
{
    const IngestResult result = use_case_->onSentence(
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47", 42.5, kNowUtc);

    EXPECT_EQ(result, IngestResult::Published);
    ASSERT_EQ(output_->fixes.size(), 1u);
    EXPECT_NEAR(output_->fixes.front().latitude_deg, 48.1173, 1e-4);
    ASSERT_EQ(output_->stamps.size(), 1u);
    EXPECT_NEAR(output_->stamps.front(), 42.5, 1e-9);
    EXPECT_EQ(output_->time_references.size(), 1u);
    EXPECT_EQ(use_case_->statistics().published, 1u);
}

TEST_F(IngestNmeaUseCaseTest, RejectsABadChecksumBeforeParsing)
{
    const IngestResult result = use_case_->onSentence(
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48", 0.0, kNowUtc);

    EXPECT_EQ(result, IngestResult::ChecksumFailed);
    EXPECT_TRUE(output_->fixes.empty());
    EXPECT_EQ(use_case_->statistics().checksum_failed, 1u);
}

TEST_F(IngestNmeaUseCaseTest, ReportsAnUnhandledSentenceAsUnparsed)
{
    const IngestResult result =
        use_case_->onSentence("$GPZDA,123519,23,03,1994,00,00*42", 0.0, kNowUtc);

    EXPECT_EQ(result, IngestResult::Unparsed);
    EXPECT_EQ(use_case_->statistics().unparsed, 1u);
}

TEST_F(IngestNmeaUseCaseTest, ReportsGstAsProducingNoOutput)
{
    const IngestResult result =
        use_case_->onSentence("$GPGST,123519,1.2,2.5,1.8,15.0,3.0,4.0,5.0*4F", 0.0, kNowUtc);

    EXPECT_EQ(result, IngestResult::NoOutput);
    EXPECT_EQ(use_case_->statistics().no_output, 1u);
}

TEST_F(IngestNmeaUseCaseTest, CarriesFixStateAcrossSentences)
{
    // VTG is gated on a valid GGA fix, so ordering matters across calls.
    EXPECT_EQ(
        use_case_->onSentence("$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48", 0.0, kNowUtc),
        IngestResult::NoOutput);
    EXPECT_TRUE(output_->velocities.empty());

    use_case_->onSentence(
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47", 0.0, kNowUtc);

    EXPECT_EQ(
        use_case_->onSentence("$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48", 0.0, kNowUtc),
        IngestResult::Published);
    EXPECT_EQ(output_->velocities.size(), 1u);
}

TEST_F(IngestNmeaUseCaseTest, ResetClearsCountersAndLatchedState)
{
    use_case_->onSentence(
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47", 0.0, kNowUtc);
    ASSERT_EQ(use_case_->statistics().published, 1u);

    use_case_->reset();

    EXPECT_EQ(use_case_->statistics().published, 0u);
    // The fix gate is closed again, so VTG alone produces nothing.
    EXPECT_EQ(
        use_case_->onSentence("$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48", 0.0, kNowUtc),
        IngestResult::NoOutput);
}

TEST(IngestNmeaUseCaseConstructionTest, RejectsANullOutputPort)
{
    EXPECT_THROW(
        IngestNmeaUseCase(domain::nmea::FixAssemblerConfig{}, nullptr), std::invalid_argument);
}
