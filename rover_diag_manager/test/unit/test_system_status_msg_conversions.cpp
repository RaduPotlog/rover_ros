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

#include <cmath>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "rover_diag_manager/infrastructure/system_status_msg_conversions.hpp"

using namespace rover_diag_manager;  // NOLINT
using namespace rover_diag_manager::domain;  // NOLINT

TEST(ToSystemStatusMsg, MapsKnownFields)
{
    SystemSample sample;
    sample.core_usages = {10.0f, 20.0f};
    sample.cpu_mean_usage = 15.0f;
    sample.cpu_temperature = 55.0f;
    sample.ram_usage = 30.0f;
    sample.disk_usage = 40.0f;

    builtin_interfaces::msg::Time stamp;
    stamp.sec = 123;

    const auto msg = infrastructure::toSystemStatusMsg(sample, stamp);

    EXPECT_EQ(msg.header.stamp.sec, 123);
    EXPECT_EQ(msg.cpu_percent, sample.core_usages);
    EXPECT_FLOAT_EQ(msg.avg_load_percent, 15.0f);
    EXPECT_FLOAT_EQ(msg.cpu_temp, 55.0f);
    EXPECT_FLOAT_EQ(msg.ram_usage_percent, 30.0f);
    EXPECT_FLOAT_EQ(msg.disc_usage_percent, 40.0f);
}

TEST(ToSystemStatusMsg, MapsUnknownMetricsToNaN)
{
    SystemSample sample;

    const auto msg = infrastructure::toSystemStatusMsg(sample, builtin_interfaces::msg::Time{});

    EXPECT_TRUE(std::isnan(msg.avg_load_percent));
    EXPECT_TRUE(std::isnan(msg.cpu_temp));
    EXPECT_TRUE(std::isnan(msg.ram_usage_percent));
    EXPECT_TRUE(std::isnan(msg.disc_usage_percent));
}

TEST(ToDiagnosticLevel, MapsEachHealthLevel)
{
    using diagnostic_msgs::msg::DiagnosticStatus;

    EXPECT_EQ(infrastructure::toDiagnosticLevel(HealthLevel::Ok), DiagnosticStatus::OK);
    EXPECT_EQ(infrastructure::toDiagnosticLevel(HealthLevel::Warn), DiagnosticStatus::WARN);
    EXPECT_EQ(infrastructure::toDiagnosticLevel(HealthLevel::Error), DiagnosticStatus::ERROR);
}

TEST(FillDiagnosticStatus, WritesOneKeyValuePerFindingAndSummary)
{
    HealthReport report;
    report.level = HealthLevel::Warn;
    report.message = "test message";
    report.findings = {
        MetricFinding{"CPU usage (%)", 42.0f, HealthLevel::Ok},
        MetricFinding{"RAM memory usage (%)", std::nullopt, HealthLevel::Error},
    };

    diagnostic_updater::DiagnosticStatusWrapper status;
    infrastructure::fillDiagnosticStatus(report, status);

    ASSERT_EQ(status.values.size(), 2u);
    EXPECT_EQ(status.values[0].key, "CPU usage (%)");
    EXPECT_EQ(status.values[0].value, "42");
    EXPECT_EQ(status.values[1].key, "RAM memory usage (%)");
    EXPECT_EQ(status.message, "test message");
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}
