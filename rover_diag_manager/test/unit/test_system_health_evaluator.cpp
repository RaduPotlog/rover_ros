// Copyright 2025 Mechatronics Academy
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

#include "rover_diag_manager/domain/system_health_evaluator.hpp"

using namespace rover_diag_manager::domain;  // NOLINT

namespace
{

SystemSample makeSample(float cpu, float temp, float ram, float disk)
{
    SystemSample sample;
    sample.core_usages = {cpu};
    sample.cpu_mean_usage = cpu;
    sample.cpu_temperature = temp;
    sample.ram_usage = ram;
    sample.disk_usage = disk;
    return sample;
}

}  // namespace

TEST(EvaluateSystemHealth, AllWithinLimitsIsOk)
{
    const auto report = evaluateSystemHealth(makeSample(10.0f, 40.0f, 20.0f, 30.0f), {});

    EXPECT_EQ(report.level, HealthLevel::Ok);
    ASSERT_EQ(report.findings.size(), 4u);
    for (const auto & finding : report.findings) {
        EXPECT_EQ(finding.level, HealthLevel::Ok);
    }
}

TEST(EvaluateSystemHealth, CpuAboveThresholdWarns)
{
    SystemHealthThresholds thresholds;
    const auto report = evaluateSystemHealth(
        makeSample(static_cast<float>(thresholds.cpu_usage) + 1.0f, 40.0f, 20.0f, 30.0f),
        thresholds);

    EXPECT_EQ(report.level, HealthLevel::Warn);
    EXPECT_EQ(report.findings[0].level, HealthLevel::Warn);
}

TEST(EvaluateSystemHealth, TemperatureAboveThresholdWarns)
{
    SystemHealthThresholds thresholds;
    const auto report = evaluateSystemHealth(
        makeSample(10.0f, static_cast<float>(thresholds.cpu_temperature) + 1.0f, 20.0f, 30.0f),
        thresholds);

    EXPECT_EQ(report.level, HealthLevel::Warn);
    EXPECT_EQ(report.findings[1].level, HealthLevel::Warn);
}

// Regression test: RAM and disk share the same default threshold (90.0). The old implementation
// keyed a map by threshold *value*, so one of these two checks silently overwrote the other.
TEST(EvaluateSystemHealth, RamAndDiskBothOverIdenticalThresholdsAreBothReported)
{
    SystemHealthThresholds thresholds;
    thresholds.ram_usage = 90.0;
    thresholds.disk_usage = 90.0;

    const auto report = evaluateSystemHealth(makeSample(10.0f, 40.0f, 95.0f, 95.0f), thresholds);

    EXPECT_EQ(report.level, HealthLevel::Warn);
    ASSERT_EQ(report.findings.size(), 4u);
    EXPECT_EQ(report.findings[2].level, HealthLevel::Warn) << "RAM finding was dropped";
    EXPECT_EQ(report.findings[3].level, HealthLevel::Warn) << "Disk finding was dropped";
}

TEST(EvaluateSystemHealth, UnknownMetricIsError)
{
    SystemSample sample = makeSample(10.0f, 40.0f, 20.0f, 30.0f);
    sample.ram_usage = std::nullopt;

    const auto report = evaluateSystemHealth(sample, {});

    EXPECT_EQ(report.level, HealthLevel::Error);
    EXPECT_EQ(report.findings[2].level, HealthLevel::Error);
    EXPECT_FALSE(report.findings[2].value.has_value());
}

TEST(EvaluateSystemHealth, ValueExactlyAtThresholdIsOk)
{
    SystemHealthThresholds thresholds;
    const auto report = evaluateSystemHealth(
        makeSample(static_cast<float>(thresholds.cpu_usage), 40.0f, 20.0f, 30.0f), thresholds);

    EXPECT_EQ(report.findings[0].level, HealthLevel::Ok);
}
