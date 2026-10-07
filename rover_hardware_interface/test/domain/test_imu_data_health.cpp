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

#include <cstdint>

#include "rover_hardware_interface/domain/imu_data_health.hpp"

namespace rover_hardware_interface
{

namespace
{
constexpr std::int64_t kS = 1000000000;  // ns per second
}

TEST(ImuDataRecorderTest, StartsEmpty)
{
    ImuDataRecorder r;
    const auto s = r.snapshot(5 * kS, 0);

    EXPECT_FALSE(s.received_any);
    EXPECT_EQ(s.total, 0u);
    EXPECT_DOUBLE_EQ(s.since_start_s, 5.0);
    EXPECT_LT(s.last_message_age_s, 0.0);
    EXPECT_LT(s.last_valid_age_s, 0.0);
}

TEST(ImuDataRecorderTest, CountsValidAndNotFiniteMessagesSeparately)
{
    ImuDataRecorder r;

    r.onMessage(true, 1 * kS);
    r.onMessage(true, 2 * kS);
    r.onMessage(false, 3 * kS);

    const auto s = r.snapshot(4 * kS, 0);
    EXPECT_EQ(s.total, 3u);
    EXPECT_EQ(s.valid, 2u);
    EXPECT_EQ(s.not_finite, 1u);
    EXPECT_FALSE(s.last_finite);
    EXPECT_DOUBLE_EQ(s.last_message_age_s, 1.0);
    EXPECT_DOUBLE_EQ(s.last_valid_age_s, 2.0);   // the last all-finite one was at 2 s
}

TEST(ImuDataHealthTest, NothingYetIsAWarningInsideTheStartupGraceThenAnError)
{
    ImuDataStats s;
    s.since_start_s = 5.0;

    EXPECT_EQ(evaluateImuDataHealth(s).level, HealthLevel::kWarn);

    s.since_start_s = 25.0;
    const auto v = evaluateImuDataHealth(s);
    EXPECT_EQ(v.level, HealthLevel::kError);
    EXPECT_NE(v.message.find("No imu/data"), std::string::npos);
}

TEST(ImuDataHealthTest, ValidFreshDataIsOk)
{
    ImuDataRecorder r;
    r.onMessage(true, 100 * kS);

    EXPECT_EQ(evaluateImuDataHealth(r.snapshot(100 * kS + kS / 10, 0)).level, HealthLevel::kOk);
}

TEST(ImuDataHealthTest, NanDataIsAnErrorEvenThoughMessagesKeepArriving)
{
    // This is exactly the unconfigured-IMU case: the broadcaster publishes at full rate, all NaN.
    ImuDataRecorder r;

    for (int i = 0; i < 25; ++i) {
        r.onMessage(false, 100 * kS + i * (kS / 25));
    }

    const auto v = evaluateImuDataHealth(r.snapshot(101 * kS, 0));
    EXPECT_EQ(v.level, HealthLevel::kError);
    EXPECT_NE(v.message.find("NaN"), std::string::npos);
}

TEST(ImuDataHealthTest, StaleDataIsAnError)
{
    ImuDataRecorder r;
    r.onMessage(true, 100 * kS);

    const auto v = evaluateImuDataHealth(r.snapshot(104 * kS, 0));
    EXPECT_EQ(v.level, HealthLevel::kError);
    EXPECT_NE(v.message.find("stale"), std::string::npos);
}

TEST(ImuDataHealthTest, RecoversWhenValidDataReturns)
{
    ImuDataRecorder r;
    r.onMessage(false, 100 * kS);
    EXPECT_EQ(evaluateImuDataHealth(r.snapshot(100 * kS, 0)).level, HealthLevel::kError);

    r.onMessage(true, 100 * kS + kS / 25);
    EXPECT_EQ(evaluateImuDataHealth(r.snapshot(100 * kS + kS / 10, 0)).level, HealthLevel::kOk);
}

TEST(ImuDataHealthTest, ThresholdsAreConfigurable)
{
    ImuDataStats s;
    s.received_any = true;
    s.last_finite = true;
    s.last_message_age_s = 0.5;

    ImuDataThresholds tight;
    tight.stale_s = 0.2;

    EXPECT_EQ(evaluateImuDataHealth(s).level, HealthLevel::kOk);
    EXPECT_EQ(evaluateImuDataHealth(s, tight).level, HealthLevel::kError);
}

}  // namespace rover_hardware_interface
