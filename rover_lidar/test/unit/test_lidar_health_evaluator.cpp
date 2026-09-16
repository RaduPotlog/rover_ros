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
#include <stdexcept>
#include <vector>

#include "rover_lidar/application/monitor_lidar_use_case.hpp"
#include "rover_lidar/domain/lidar_health_evaluator.hpp"
#include "rover_lidar/domain/ports/lidar_health_publisher_port.hpp"

using rover_lidar::domain::CloudSample;
using rover_lidar::domain::HealthLevel;
using rover_lidar::domain::LidarHealthEvaluator;
using rover_lidar::domain::LidarHealthThresholds;

namespace
{

LidarHealthThresholds defaultThresholds()
{
    LidarHealthThresholds thresholds;
    thresholds.expected_rate_hz = 10.0;
    thresholds.min_rate_ratio = 0.5;
    thresholds.cloud_timeout_s = 2.0;
    thresholds.min_points_warn = 1000;
    return thresholds;
}

CloudSample cloudAt(double arrival_s, std::uint32_t points = 28800)
{
    CloudSample cloud;
    cloud.arrival_s = arrival_s;
    cloud.point_count = points;
    return cloud;
}

/** @brief Feeds `count` clouds at the nominal rate, ending at t = count * period. */
void feedHealthyStream(LidarHealthEvaluator & evaluator, int count, double period_s = 0.1)
{
    for (int i = 1; i <= count; ++i) {
        evaluator.addCloud(cloudAt(i * period_s));
    }
}

class RecordingPublisher : public rover_lidar::domain::LidarHealthPublisherPort
{
public:
    void publish(const rover_lidar::domain::LidarHealthReport & report) override
    {
        reports.push_back(report);
    }

    std::vector<rover_lidar::domain::LidarHealthReport> reports;
};

}  // namespace

TEST(LidarHealthEvaluatorTest, StaleBeforeAnyCloud)
{
    LidarHealthEvaluator evaluator(defaultThresholds());

    const auto report = evaluator.evaluate(0.0);

    EXPECT_EQ(report.level, HealthLevel::Stale);
    EXPECT_FALSE(report.has_data);
    EXPECT_EQ(report.cloud_count, 0u);
}

TEST(LidarHealthEvaluatorTest, OkOnNominalStream)
{
    LidarHealthEvaluator evaluator(defaultThresholds());
    feedHealthyStream(evaluator, 20);

    const auto report = evaluator.evaluate(2.0);

    EXPECT_EQ(report.level, HealthLevel::Ok);
    EXPECT_TRUE(report.has_data);
    EXPECT_NEAR(report.rate_hz, 10.0, 0.5);
    EXPECT_EQ(report.cloud_count, 20u);
}

TEST(LidarHealthEvaluatorTest, SingleCloudIsOkNotSlow)
{
    // One cloud gives no rate yet; that must not be reported as "rate too low".
    LidarHealthEvaluator evaluator(defaultThresholds());
    evaluator.addCloud(cloudAt(1.0));

    const auto report = evaluator.evaluate(1.0);

    EXPECT_EQ(report.level, HealthLevel::Ok);
    EXPECT_EQ(report.rate_hz, 0.0);
}

TEST(LidarHealthEvaluatorTest, ErrorWhenStreamTimesOut)
{
    LidarHealthEvaluator evaluator(defaultThresholds());
    feedHealthyStream(evaluator, 20);

    // Last cloud at t = 2.0, evaluated well past cloud_timeout_s.
    const auto report = evaluator.evaluate(5.0);

    EXPECT_EQ(report.level, HealthLevel::Error);
    EXPECT_NEAR(report.age_s, 3.0, 1e-9);
}

TEST(LidarHealthEvaluatorTest, TimeoutOutranksSparseCloud)
{
    LidarHealthEvaluator evaluator(defaultThresholds());
    evaluator.addCloud(cloudAt(1.0, 5));

    const auto report = evaluator.evaluate(10.0);

    EXPECT_EQ(report.level, HealthLevel::Error);
}

TEST(LidarHealthEvaluatorTest, WarnOnSparseCloud)
{
    LidarHealthEvaluator evaluator(defaultThresholds());
    for (int i = 1; i <= 20; ++i) {
        evaluator.addCloud(cloudAt(i * 0.1, 10));
    }

    const auto report = evaluator.evaluate(2.0);

    EXPECT_EQ(report.level, HealthLevel::Warn);
    EXPECT_EQ(report.point_count, 10u);
}

TEST(LidarHealthEvaluatorTest, WarnOnSlowStream)
{
    LidarHealthEvaluator evaluator(defaultThresholds());
    // 3 Hz against an expected 10 Hz, i.e. below the 0.5 ratio, but inside cloud_timeout_s.
    feedHealthyStream(evaluator, 6, 1.0 / 3.0);

    const auto report = evaluator.evaluate(2.0);

    EXPECT_EQ(report.level, HealthLevel::Warn);
    EXPECT_LT(report.rate_hz, 5.0);
}

TEST(LidarHealthEvaluatorTest, RateDecaysWhileStreamIsPaused)
{
    LidarHealthEvaluator evaluator(defaultThresholds());
    feedHealthyStream(evaluator, 20);

    // Measured up to `now`, so a stalled stream must not keep reporting its old rate.
    EXPECT_LT(evaluator.evaluate(3.0).rate_hz, evaluator.evaluate(2.0).rate_hz);
}

TEST(LidarHealthEvaluatorTest, RecoversAfterTimeout)
{
    LidarHealthEvaluator evaluator(defaultThresholds());
    feedHealthyStream(evaluator, 20);
    ASSERT_EQ(evaluator.evaluate(5.0).level, HealthLevel::Error);

    for (int i = 1; i <= 20; ++i) {
        evaluator.addCloud(cloudAt(5.0 + i * 0.1));
    }

    EXPECT_EQ(evaluator.evaluate(7.0).level, HealthLevel::Ok);
}

TEST(LidarHealthEvaluatorTest, RejectsNonPositiveThresholds)
{
    auto thresholds = defaultThresholds();
    thresholds.expected_rate_hz = 0.0;
    EXPECT_THROW(LidarHealthEvaluator::validate(thresholds), std::invalid_argument);

    thresholds = defaultThresholds();
    thresholds.cloud_timeout_s = -1.0;
    EXPECT_THROW(LidarHealthEvaluator::validate(thresholds), std::invalid_argument);

    EXPECT_NO_THROW(LidarHealthEvaluator::validate(defaultThresholds()));
}

TEST(MonitorLidarUseCaseTest, PublishesEvaluatedReport)
{
    auto publisher = std::make_shared<RecordingPublisher>();
    rover_lidar::application::MonitorLidarUseCase use_case(defaultThresholds(), publisher);

    use_case.publishHealth(0.0);
    ASSERT_EQ(publisher->reports.size(), 1u);
    EXPECT_EQ(publisher->reports.back().level, HealthLevel::Stale);

    for (int i = 1; i <= 20; ++i) {
        use_case.onCloud(cloudAt(i * 0.1));
    }
    const auto report = use_case.publishHealth(2.0);

    ASSERT_EQ(publisher->reports.size(), 2u);
    EXPECT_EQ(publisher->reports.back().level, HealthLevel::Ok);
    EXPECT_EQ(report.level, HealthLevel::Ok);
}

TEST(MonitorLidarUseCaseTest, RequiresAPublisher)
{
    EXPECT_THROW(
        rover_lidar::application::MonitorLidarUseCase(defaultThresholds(), nullptr),
        std::invalid_argument);
}
