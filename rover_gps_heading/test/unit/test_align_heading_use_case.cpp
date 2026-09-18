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
#include <memory>
#include <stdexcept>
#include <vector>

#include "rover_gps_heading/application/align_heading_use_case.hpp"

using rover_gps_heading::application::AlignHeadingSettings;
using rover_gps_heading::application::AlignHeadingUseCase;
using namespace rover_gps_heading::domain;  // NOLINT

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;

class FakeHeadingPublisher : public HeadingPublisherPort
{
public:
    void publishHeading(const EnuHeading & heading) override {headings.push_back(heading);}

    void publishAlignmentStatus(const AlignmentStatus & status) override
    {
        statuses.push_back(status);
    }

    std::vector<EnuHeading> headings;
    std::vector<AlignmentStatus> statuses;
};

GnssFix fixAtNorth(double north_m, double stamp_s)
{
    GnssFix fix;
    fix.latitude_deg = 45.0 + north_m / kEarthRadiusM * 180.0 / kPi;
    fix.longitude_deg = 25.0;
    fix.status = FixStatus::Fix;
    fix.horizontal_std_m = 1.0;
    fix.stamp_s = stamp_s;
    return fix;
}

OdometrySample odomAt(double stamp_s, double yaw_rad = 0.0)
{
    OdometrySample sample;
    sample.stamp_s = stamp_s;
    sample.yaw_rad = yaw_rad;
    sample.vx_m_s = 1.0;
    return sample;
}

/** @brief Drives north at 1 m/s with odom yaw 0 until the use case is aligned (offset +pi/2). */
void driveNorthUntilAligned(AlignHeadingUseCase & use_case)
{
    for (int second = 0; second <= 20 && !use_case.aligned(); ++second) {
        use_case.onOdometry(odomAt(second));
        use_case.onFix(fixAtNorth(second, second));
    }
}

}  // namespace

TEST(AlignHeadingUseCaseTest, PublishesEnuHeadingOnceAligned)
{
    auto publisher = std::make_shared<FakeHeadingPublisher>();
    AlignHeadingSettings settings;
    settings.publish_heading = true;
    AlignHeadingUseCase use_case(publisher, HeadingAlignmentConfig{}, settings);

    use_case.onOdometry(odomAt(0.0));
    EXPECT_TRUE(publisher->headings.empty());

    driveNorthUntilAligned(use_case);
    ASSERT_TRUE(use_case.aligned());
    EXPECT_TRUE(publisher->headings.empty());

    use_case.onOdometry(odomAt(30.0, 0.25));
    ASSERT_EQ(publisher->headings.size(), 1u);
    EXPECT_NEAR(publisher->headings[0].yaw_rad, kPi / 2.0 + 0.25, 0.01);
    EXPECT_DOUBLE_EQ(publisher->headings[0].yaw_std_rad, settings.min_heading_std_rad);
    EXPECT_DOUBLE_EQ(publisher->headings[0].stamp_s, 30.0);
}

TEST(AlignHeadingUseCaseTest, HeadingOutputDisabled)
{
    auto publisher = std::make_shared<FakeHeadingPublisher>();
    AlignHeadingUseCase use_case(publisher, HeadingAlignmentConfig{}, AlignHeadingSettings{});

    driveNorthUntilAligned(use_case);
    ASSERT_TRUE(use_case.aligned());
    use_case.onOdometry(odomAt(30.0));

    EXPECT_TRUE(publisher->headings.empty());
}

TEST(AlignHeadingUseCaseTest, TickAndResetPublishStatus)
{
    auto publisher = std::make_shared<FakeHeadingPublisher>();
    AlignHeadingUseCase use_case(publisher, HeadingAlignmentConfig{}, AlignHeadingSettings{});

    driveNorthUntilAligned(use_case);
    use_case.onTick();
    use_case.reset();

    ASSERT_EQ(publisher->statuses.size(), 2u);
    EXPECT_EQ(publisher->statuses[0].state, AlignmentState::Aligned);
    EXPECT_EQ(publisher->statuses[1].state, AlignmentState::WaitingForMotion);
    EXPECT_FALSE(use_case.aligned());
}

TEST(AlignHeadingUseCaseTest, RejectsInvalidSettings)
{
    AlignHeadingSettings settings;
    settings.min_heading_std_rad = 0.0;
    EXPECT_THROW(
        AlignHeadingUseCase(
            std::make_shared<FakeHeadingPublisher>(), HeadingAlignmentConfig{}, settings),
        std::invalid_argument);
}
