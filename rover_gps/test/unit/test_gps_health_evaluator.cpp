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

#include <limits>
#include <stdexcept>

#include "rover_gps/domain/gps_health_evaluator.hpp"

using namespace rover_gps::domain;  // NOLINT

namespace
{

GnssFix makeFix(double stamp_s, FixStatus status = FixStatus::Fix, double std_m = 2.0)
{
    GnssFix fix;
    fix.latitude_deg = 45.0;
    fix.longitude_deg = 25.0;
    fix.status = status;
    fix.horizontal_std_m = std_m;
    fix.stamp_s = stamp_s;
    return fix;
}

}  // namespace

TEST(GpsHealthEvaluatorTest, StaleBeforeFirstFix)
{
    const GpsHealthEvaluator evaluator(GpsHealthThresholds{});
    const GpsHealthReport report = evaluator.evaluate(10.0);
    EXPECT_EQ(report.level, HealthLevel::Stale);
    EXPECT_FALSE(report.has_data);
}

TEST(GpsHealthEvaluatorTest, OkAtExpectedRate)
{
    GpsHealthEvaluator evaluator(GpsHealthThresholds{});
    for (int i = 0; i < 5; ++i) {
        evaluator.addFix(makeFix(100.0 + i));
    }
    const GpsHealthReport report = evaluator.evaluate(104.2);
    EXPECT_EQ(report.level, HealthLevel::Ok);
    EXPECT_NEAR(report.rate_hz, 4.0 / 4.2, 1e-9);
    EXPECT_EQ(report.fix_count, 5u);
}

TEST(GpsHealthEvaluatorTest, SingleFixIsOk)
{
    GpsHealthEvaluator evaluator(GpsHealthThresholds{});
    evaluator.addFix(makeFix(100.0));
    EXPECT_EQ(evaluator.evaluate(100.5).level, HealthLevel::Ok);
}

TEST(GpsHealthEvaluatorTest, TimeoutIsError)
{
    GpsHealthEvaluator evaluator(GpsHealthThresholds{});
    evaluator.addFix(makeFix(100.0));
    const GpsHealthReport report = evaluator.evaluate(103.5);
    EXPECT_EQ(report.level, HealthLevel::Error);
    EXPECT_NEAR(report.age_s, 3.5, 1e-9);
}

TEST(GpsHealthEvaluatorTest, NoFixIsWarn)
{
    GpsHealthEvaluator evaluator(GpsHealthThresholds{});
    evaluator.addFix(makeFix(100.0, FixStatus::NoFix, 1.0e6));
    EXPECT_EQ(evaluator.evaluate(100.1).level, HealthLevel::Warn);
}

TEST(GpsHealthEvaluatorTest, AccuracyThresholds)
{
    GpsHealthEvaluator degraded(GpsHealthThresholds{});
    degraded.addFix(makeFix(100.0, FixStatus::Fix, 8.0));
    EXPECT_EQ(degraded.evaluate(100.1).level, HealthLevel::Warn);

    GpsHealthEvaluator poor(GpsHealthThresholds{});
    poor.addFix(makeFix(100.0, FixStatus::Fix, 25.0));
    EXPECT_EQ(poor.evaluate(100.1).level, HealthLevel::Error);

    GpsHealthEvaluator unknown(GpsHealthThresholds{});
    unknown.addFix(makeFix(100.0, FixStatus::Fix, std::numeric_limits<double>::quiet_NaN()));
    EXPECT_EQ(unknown.evaluate(100.1).level, HealthLevel::Ok);
}

TEST(GpsHealthEvaluatorTest, LowRateIsWarn)
{
    GpsHealthThresholds thresholds;
    thresholds.expected_rate_hz = 5.0;
    GpsHealthEvaluator evaluator(thresholds);
    evaluator.addFix(makeFix(100.0));
    evaluator.addFix(makeFix(101.0));
    evaluator.addFix(makeFix(102.0));
    EXPECT_EQ(evaluator.evaluate(102.1).level, HealthLevel::Warn);
}

TEST(GpsHealthEvaluatorTest, RejectsInvalidThresholds)
{
    GpsHealthThresholds thresholds;
    thresholds.warn_horizontal_std_m = 30.0;
    EXPECT_THROW(GpsHealthEvaluator{thresholds}, std::invalid_argument);

    GpsHealthThresholds zero_rate;
    zero_rate.expected_rate_hz = 0.0;
    EXPECT_THROW(GpsHealthEvaluator{zero_rate}, std::invalid_argument);
}
