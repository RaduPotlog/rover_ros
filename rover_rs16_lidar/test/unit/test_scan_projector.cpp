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

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "rover_rs16_lidar/domain/scan_projector.hpp"

namespace
{

using rover_rs16_lidar::domain::LidarPoint;
using rover_rs16_lidar::domain::PointCloudFrame;
using rover_rs16_lidar::domain::ScanProjector;
using rover_rs16_lidar::domain::ScanSettings;

constexpr double kPi = 3.141592653589793;

/// A coarse 4-bin sweep, so a bin index can be reasoned about by hand.
ScanSettings quarterTurnSettings()
{
    ScanSettings settings;
    settings.min_height = -0.25;
    settings.max_height = 0.25;
    settings.angle_min = -kPi;
    settings.angle_max = kPi;
    settings.angle_increment = kPi / 2.0;
    settings.range_min = 0.2;
    settings.range_max = 20.0;
    settings.scan_time = 0.1;
    settings.use_inf = true;
    return settings;
}

PointCloudFrame cloudOf(std::initializer_list<LidarPoint> points)
{
    PointCloudFrame cloud;
    cloud.points = points;
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.timestamp_s = 1234.5;
    return cloud;
}

/// Bin a bearing falls into, using the projector's own arithmetic.
std::size_t binOf(const ScanSettings & settings, double angle)
{
    return static_cast<std::size_t>((angle - settings.angle_min) / settings.angle_increment);
}

}  // namespace

TEST(ScanProjector, RejectsSettingsThatCannotProduceAScan)
{
    auto settings = quarterTurnSettings();
    settings.min_height = settings.max_height;
    EXPECT_THROW(ScanProjector{settings}, std::invalid_argument);

    settings = quarterTurnSettings();
    settings.angle_increment = 0.0;
    EXPECT_THROW(ScanProjector{settings}, std::invalid_argument);

    settings = quarterTurnSettings();
    settings.range_min = settings.range_max;
    EXPECT_THROW(ScanProjector{settings}, std::invalid_argument);
}

TEST(ScanProjector, BinCountCoversTheWholeSweep)
{
    const ScanProjector projector{quarterTurnSettings()};
    EXPECT_EQ(projector.binCount(), 4U);

    // The RS16 default: a full turn at half a degree.
    auto settings = quarterTurnSettings();
    settings.angle_increment = 0.008726646259971648;
    const ScanProjector fine{settings};
    EXPECT_EQ(fine.binCount(), 720U);
}

TEST(ScanProjector, EmptyCloudIsAllInfinity)
{
    const ScanProjector projector{quarterTurnSettings()};

    const auto scan = projector.project(PointCloudFrame{});

    ASSERT_EQ(scan.ranges.size(), 4U);
    for (const float range : scan.ranges) {
        EXPECT_TRUE(std::isinf(range));
    }
}

TEST(ScanProjector, EmptyBinsUseRangeMaxPlusOneWhenUseInfIsOff)
{
    auto settings = quarterTurnSettings();
    settings.use_inf = false;
    const ScanProjector projector{settings};

    const auto scan = projector.project(PointCloudFrame{});

    for (const float range : scan.ranges) {
        EXPECT_FLOAT_EQ(range, static_cast<float>(settings.range_max + 1.0));
    }
}

TEST(ScanProjector, CopiesTheCloudTimestampAndScanGeometry)
{
    const auto settings = quarterTurnSettings();
    const ScanProjector projector{settings};

    const auto scan = projector.project(cloudOf({LidarPoint{1.0F, 0.0F, 0.0F, 5.0F}}));

    EXPECT_DOUBLE_EQ(scan.timestamp_s, 1234.5);
    EXPECT_DOUBLE_EQ(scan.angle_min, settings.angle_min);
    EXPECT_DOUBLE_EQ(scan.angle_max, settings.angle_max);
    EXPECT_DOUBLE_EQ(scan.angle_increment, settings.angle_increment);
    EXPECT_DOUBLE_EQ(scan.scan_time, settings.scan_time);
    EXPECT_DOUBLE_EQ(scan.range_min, settings.range_min);
    EXPECT_DOUBLE_EQ(scan.range_max, settings.range_max);
    // The cloud is not ordered by bearing, so there is no per-beam offset to report.
    EXPECT_DOUBLE_EQ(scan.time_increment, 0.0);
}

TEST(ScanProjector, PlacesAPointInTheBinMatchingItsBearing)
{
    const auto settings = quarterTurnSettings();
    const ScanProjector projector{settings};

    // Straight ahead, 3 m out, inside the height band.
    const auto scan = projector.project(cloudOf({LidarPoint{3.0F, 0.0F, 0.1F, 7.0F}}));

    const auto index = binOf(settings, 0.0);
    ASSERT_LT(index, scan.ranges.size());
    EXPECT_FLOAT_EQ(scan.ranges[index], 3.0F);
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        if (i != index) {
            EXPECT_TRUE(std::isinf(scan.ranges[i])) << "bin " << i;
        }
    }
}

TEST(ScanProjector, KeepsTheClosestReturnInABin)
{
    const auto settings = quarterTurnSettings();
    const ScanProjector projector{settings};

    // Both bear ~0 rad, so they compete for the same bin; the far one must not win.
    const auto scan = projector.project(cloudOf({
        LidarPoint{9.0F, 0.0F, 0.0F, 1.0F},
        LidarPoint{2.0F, 0.0F, 0.0F, 1.0F},
        LidarPoint{5.0F, 0.0F, 0.0F, 1.0F},
    }));

    EXPECT_FLOAT_EQ(scan.ranges[binOf(settings, 0.0)], 2.0F);
}

TEST(ScanProjector, DropsPointsOutsideTheHeightBand)
{
    const auto settings = quarterTurnSettings();
    const ScanProjector projector{settings};

    const auto scan = projector.project(cloudOf({
        LidarPoint{3.0F, 0.0F, 0.9F, 1.0F},   // above max_height
        LidarPoint{4.0F, 0.0F, -0.9F, 1.0F},  // below min_height
    }));

    for (const float range : scan.ranges) {
        EXPECT_TRUE(std::isinf(range));
    }
}

TEST(ScanProjector, DropsPointsOutsideTheRangeWindow)
{
    const auto settings = quarterTurnSettings();
    const ScanProjector projector{settings};

    const auto scan = projector.project(cloudOf({
        LidarPoint{0.1F, 0.0F, 0.0F, 1.0F},    // closer than range_min
        LidarPoint{50.0F, 0.0F, 0.0F, 1.0F},   // further than range_max
    }));

    for (const float range : scan.ranges) {
        EXPECT_TRUE(std::isinf(range));
    }
}

TEST(ScanProjector, DropsNonFinitePoints)
{
    const auto settings = quarterTurnSettings();
    const ScanProjector projector{settings};

    // dense_points is false on the rover, so invalid returns really do arrive as NaN.
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    const auto inf = std::numeric_limits<float>::infinity();
    const auto scan = projector.project(cloudOf({
        LidarPoint{nan, nan, nan, 0.0F},
        LidarPoint{3.0F, nan, 0.0F, 0.0F},
        LidarPoint{inf, 0.0F, 0.0F, 0.0F},
        LidarPoint{3.0F, 0.0F, nan, 0.0F},
    }));

    for (const float range : scan.ranges) {
        EXPECT_TRUE(std::isinf(range));
    }
}

TEST(ScanProjector, FoldsABearingOfExactlyAngleMaxIntoTheLastBin)
{
    const auto settings = quarterTurnSettings();
    const ScanProjector projector{settings};

    // atan2(0, -x) is exactly +pi, i.e. exactly angle_max, which divides out to one past the
    // last bin. Directly behind the rover must not be a blind spot.
    const auto scan = projector.project(cloudOf({LidarPoint{-5.0F, 0.0F, 0.0F, 1.0F}}));

    ASSERT_EQ(scan.ranges.size(), 4U);
    EXPECT_FLOAT_EQ(scan.ranges.back(), 5.0F);
}

TEST(ScanProjector, DropsPointsOutsideTheAngleWindow)
{
    auto settings = quarterTurnSettings();
    // Forward-facing quadrant only.
    settings.angle_min = -kPi / 4.0;
    settings.angle_max = kPi / 4.0;
    settings.angle_increment = kPi / 8.0;
    const ScanProjector projector{settings};

    const auto scan = projector.project(cloudOf({
        LidarPoint{0.0F, 3.0F, 0.0F, 1.0F},   // +pi/2, outside
        LidarPoint{-3.0F, 0.0F, 0.0F, 1.0F},  // pi, outside
        LidarPoint{3.0F, 0.0F, 0.0F, 1.0F},   // 0, inside
    }));

    ASSERT_EQ(scan.ranges.size(), 4U);
    EXPECT_FLOAT_EQ(scan.ranges[binOf(settings, 0.0)], 3.0F);
}

TEST(ScanProjector, ReusesNothingBetweenProjections)
{
    const auto settings = quarterTurnSettings();
    const ScanProjector projector{settings};

    const auto first = projector.project(cloudOf({LidarPoint{3.0F, 0.0F, 0.0F, 1.0F}}));
    ASSERT_FLOAT_EQ(first.ranges[binOf(settings, 0.0)], 3.0F);

    // A second, empty revolution must not keep reporting the first one's obstacle.
    const auto second = projector.project(PointCloudFrame{});
    for (const float range : second.ranges) {
        EXPECT_TRUE(std::isinf(range));
    }
}
