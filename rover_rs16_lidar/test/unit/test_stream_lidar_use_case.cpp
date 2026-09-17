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

#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "rover_rs16_lidar/application/stream_lidar_use_case.hpp"

namespace
{

using rover_rs16_lidar::application::MonitorLidarUseCase;
using rover_rs16_lidar::application::StreamLidarUseCase;
using rover_rs16_lidar::domain::LaserScanFrame;
using rover_rs16_lidar::domain::LaserScanPublisherPort;
using rover_rs16_lidar::domain::LidarHealthReport;
using rover_rs16_lidar::domain::LidarHealthPublisherPort;
using rover_rs16_lidar::domain::LidarPoint;
using rover_rs16_lidar::domain::PointCloudFrame;
using rover_rs16_lidar::domain::PointCloudPublisherPort;
using rover_rs16_lidar::domain::ScanProjector;
using rover_rs16_lidar::domain::ScanSettings;

class SpyCloudPublisher : public PointCloudPublisherPort
{
public:
    void publish(const PointCloudFrame & cloud) override { clouds.push_back(cloud); }
    std::vector<PointCloudFrame> clouds;
};

class SpyScanPublisher : public LaserScanPublisherPort
{
public:
    void publish(const LaserScanFrame & scan) override { scans.push_back(scan); }
    std::vector<LaserScanFrame> scans;
};

class SpyHealthPublisher : public LidarHealthPublisherPort
{
public:
    void publish(const LidarHealthReport & report) override { reports.push_back(report); }
    std::vector<LidarHealthReport> reports;
};

std::shared_ptr<ScanProjector> defaultProjector()
{
    return std::make_shared<ScanProjector>(ScanSettings{});
}

PointCloudFrame twoPointCloud()
{
    PointCloudFrame cloud;
    cloud.points = {LidarPoint{3.0F, 0.0F, 0.0F, 1.0F}, LidarPoint{0.0F, 3.0F, 0.0F, 1.0F}};
    cloud.width = 2;
    cloud.height = 1;
    cloud.timestamp_s = 10.0;
    return cloud;
}

}  // namespace

TEST(StreamLidarUseCase, RequiresACloudPublisherAndAMonitor)
{
    auto monitor = std::make_shared<MonitorLidarUseCase>(
        rover_rs16_lidar::domain::LidarHealthThresholds{}, std::make_shared<SpyHealthPublisher>());

    EXPECT_THROW(
        StreamLidarUseCase(nullptr, nullptr, nullptr, monitor), std::invalid_argument);
    EXPECT_THROW(
        StreamLidarUseCase(std::make_shared<SpyCloudPublisher>(), nullptr, nullptr, nullptr),
        std::invalid_argument);
}

TEST(StreamLidarUseCase, RefusesToProjectWithoutSomewhereToPublishTheScan)
{
    auto monitor = std::make_shared<MonitorLidarUseCase>(
        rover_rs16_lidar::domain::LidarHealthThresholds{}, std::make_shared<SpyHealthPublisher>());

    EXPECT_THROW(
        StreamLidarUseCase(
            std::make_shared<SpyCloudPublisher>(), nullptr, defaultProjector(), monitor),
        std::invalid_argument);
}

TEST(StreamLidarUseCase, PublishesCloudAndScanAndFeedsTheMonitor)
{
    auto clouds = std::make_shared<SpyCloudPublisher>();
    auto scans = std::make_shared<SpyScanPublisher>();
    auto health = std::make_shared<SpyHealthPublisher>();
    auto monitor = std::make_shared<MonitorLidarUseCase>(
        rover_rs16_lidar::domain::LidarHealthThresholds{}, health);

    StreamLidarUseCase use_case(clouds, scans, defaultProjector(), monitor);
    use_case.onFrame(twoPointCloud(), 42.0);

    ASSERT_EQ(clouds->clouds.size(), 1U);
    EXPECT_EQ(clouds->clouds.front().points.size(), 2U);
    ASSERT_EQ(scans->scans.size(), 1U);
    EXPECT_FALSE(scans->scans.front().ranges.empty());

    // The monitor saw the cloud: the report is no longer STALE and counts both points.
    const auto report = monitor->publishHealth(42.0);
    EXPECT_TRUE(report.has_data);
    EXPECT_EQ(report.point_count, 2U);
    EXPECT_EQ(report.cloud_count, 1U);
}

TEST(StreamLidarUseCase, SkipsTheProjectionWhenNoScanIsWanted)
{
    auto clouds = std::make_shared<SpyCloudPublisher>();
    auto scans = std::make_shared<SpyScanPublisher>();
    auto monitor = std::make_shared<MonitorLidarUseCase>(
        rover_rs16_lidar::domain::LidarHealthThresholds{}, std::make_shared<SpyHealthPublisher>());

    StreamLidarUseCase use_case(clouds, scans, nullptr, monitor);
    use_case.onFrame(twoPointCloud(), 1.0);

    EXPECT_EQ(clouds->clouds.size(), 1U);
    EXPECT_TRUE(scans->scans.empty());
}

TEST(StreamLidarUseCase, UsesArrivalTimeRatherThanTheCloudStamp)
{
    auto monitor = std::make_shared<MonitorLidarUseCase>(
        rover_rs16_lidar::domain::LidarHealthThresholds{}, std::make_shared<SpyHealthPublisher>());
    StreamLidarUseCase use_case(
        std::make_shared<SpyCloudPublisher>(), nullptr, nullptr, monitor);

    // The cloud claims t=10 (a lidar clock), but it arrived at t=100 on the rover clock.
    use_case.onFrame(twoPointCloud(), 100.0);

    // Ageing from arrival, the cloud is fresh at t=100 - not 90 s stale.
    const auto report = monitor->publishHealth(100.0);
    EXPECT_DOUBLE_EQ(report.age_s, 0.0);
}
