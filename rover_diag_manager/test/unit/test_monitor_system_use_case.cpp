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

#include <memory>
#include <stdexcept>
#include <vector>

#include "rover_diag_manager/application/monitor_system_use_case.hpp"

using rover_diag_manager::application::MonitorSystemUseCase;
using namespace rover_diag_manager::domain;  // NOLINT

namespace
{

class FakeMetricsSource : public SystemMetricsSourcePort
{
public:
    SystemSample sample() override
    {
        ++sample_calls;

        SystemSample s;
        s.core_usages = {42.0f};
        s.cpu_mean_usage = 42.0f;
        s.cpu_temperature = 50.0f;
        s.ram_usage = 60.0f;
        s.disk_usage = 70.0f;
        return s;
    }

    int sample_calls{0};
};

class FakePublisher : public SystemStatusPublisherPort
{
public:
    void publish(const SystemSample & sample, const HealthReport & health) override
    {
        samples.push_back(sample);
        reports.push_back(health);
    }

    std::vector<SystemSample> samples;
    std::vector<HealthReport> reports;
};

class MonitorSystemUseCaseTest : public ::testing::Test
{
protected:
    std::shared_ptr<FakeMetricsSource> metrics_source_ = std::make_shared<FakeMetricsSource>();
    std::shared_ptr<FakePublisher> publisher_ = std::make_shared<FakePublisher>();
    MonitorSystemUseCase use_case_{metrics_source_, publisher_};
};

}  // namespace

TEST(MonitorSystemUseCaseCtor, RejectsNullMetricsSource)
{
    EXPECT_THROW(
        MonitorSystemUseCase(nullptr, std::make_shared<FakePublisher>()), std::invalid_argument);
}

TEST(MonitorSystemUseCaseCtor, RejectsNullPublisher)
{
    EXPECT_THROW(
        MonitorSystemUseCase(std::make_shared<FakeMetricsSource>(), nullptr),
        std::invalid_argument);
}

// Regression test: the old diagSystem() re-sampled the system, so one timer tick could sample
// twice (once for the published message, once for the diagnostic check).
TEST_F(MonitorSystemUseCaseTest, SamplesExactlyOncePerTick)
{
    use_case_.tick(SystemHealthThresholds{});

    EXPECT_EQ(metrics_source_->sample_calls, 1);
}

TEST_F(MonitorSystemUseCaseTest, PublishesSampleAndReportFromThatTick)
{
    SystemHealthThresholds low;
    low.cpu_usage = 1.0;

    use_case_.tick(low);

    ASSERT_EQ(publisher_->samples.size(), 1u);
    EXPECT_FLOAT_EQ(*publisher_->samples.front().cpu_mean_usage, 42.0f);

    ASSERT_EQ(publisher_->reports.size(), 1u);
    EXPECT_EQ(publisher_->reports.front().level, HealthLevel::Warn);
}

TEST_F(MonitorSystemUseCaseTest, DoesNotCacheThresholdsAcrossTicks)
{
    SystemHealthThresholds high;
    high.cpu_usage = 100.0;
    use_case_.tick(high);
    EXPECT_EQ(publisher_->reports.back().level, HealthLevel::Ok);

    SystemHealthThresholds low;
    low.cpu_usage = 1.0;
    use_case_.tick(low);
    EXPECT_EQ(publisher_->reports.back().level, HealthLevel::Warn);
}
