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
#include <limits>

#include "rover_hardware_interface/domain/command_stats.hpp"

namespace rover_hardware_interface
{

namespace
{
constexpr std::int64_t kMs = 1000000;  // ns per ms
}

TEST(CommandStatsRecorderTest, StartsAtZero)
{
    CommandStatsRecorder r;
    const auto s = r.snapshot(false, 0);

    EXPECT_EQ(s.submitted, 0u);
    EXPECT_EQ(s.attempted(), 0u);
    EXPECT_EQ(s.completed(), 0u);
    EXPECT_DOUBLE_EQ(s.meanLatencyUs(), 0.0);
    EXPECT_DOUBLE_EQ(s.droppedPendingRatio(), 0.0);
    EXPECT_FALSE(s.pending);
    EXPECT_EQ(s.pending_age_ms, 0u);
}

TEST(CommandStatsRecorderTest, SubmitThenOkCompletionMeasuresLatency)
{
    CommandStatsRecorder r;

    r.onSubmit(100 * kMs);
    r.onCompletion(CommandCompletion::kOk, 0, 103 * kMs);  // 3 ms later

    const auto s = r.snapshot(false, 200 * kMs);
    EXPECT_EQ(s.submitted, 1u);
    EXPECT_EQ(s.completed_ok, 1u);
    EXPECT_EQ(s.completed_failsafe, 0u);
    EXPECT_EQ(s.completed_error, 0u);
    EXPECT_EQ(s.latency_last_us, 3000u);
    EXPECT_EQ(s.latency_max_us, 3000u);
    EXPECT_DOUBLE_EQ(s.meanLatencyUs(), 3000.0);
    EXPECT_EQ(s.last_error_code, 0);
}

TEST(CommandStatsRecorderTest, LatencyMaxAndMeanTrackSeveralCompletions)
{
    CommandStatsRecorder r;

    r.onSubmit(0);
    r.onCompletion(CommandCompletion::kOk, 0, 2 * kMs);   // 2 ms
    r.onSubmit(10 * kMs);
    r.onCompletion(CommandCompletion::kOk, 0, 18 * kMs);  // 8 ms
    r.onSubmit(20 * kMs);
    r.onCompletion(CommandCompletion::kOk, 0, 24 * kMs);  // 4 ms

    const auto s = r.snapshot(false, 30 * kMs);
    EXPECT_EQ(s.latency_max_us, 8000u);
    EXPECT_EQ(s.latency_last_us, 4000u);
    EXPECT_DOUBLE_EQ(s.meanLatencyUs(), (2000.0 + 8000.0 + 4000.0) / 3.0);
}

TEST(CommandStatsRecorderTest, FailsafeAndOtherErrorsAreCountedSeparatelyAndKeepTheirCodes)
{
    CommandStatsRecorder r;

    r.onSubmit(0);
    r.onCompletion(CommandCompletion::kFailsafe, 59, kMs);
    EXPECT_EQ(r.snapshot(false, kMs).last_error_code, 59);

    r.onSubmit(2 * kMs);
    r.onCompletion(CommandCompletion::kError, 5, 3 * kMs);

    const auto s = r.snapshot(false, 4 * kMs);
    EXPECT_EQ(s.completed_failsafe, 1u);
    EXPECT_EQ(s.completed_error, 1u);
    EXPECT_EQ(s.completed_ok, 0u);
    EXPECT_EQ(s.completed(), 2u);
    EXPECT_EQ(s.last_error_code, 5);
}

TEST(CommandStatsRecorderTest, OkCompletionDoesNotOverwriteTheLastErrorCode)
{
    CommandStatsRecorder r;

    r.onSubmit(0);
    r.onCompletion(CommandCompletion::kError, 7, kMs);
    r.onSubmit(2 * kMs);
    r.onCompletion(CommandCompletion::kOk, 0, 3 * kMs);

    EXPECT_EQ(r.snapshot(false, 4 * kMs).last_error_code, 7);
}

TEST(CommandStatsRecorderTest, DropsAreCountedAndFormTheDroppedPendingRatio)
{
    CommandStatsRecorder r;

    r.onSubmit(0);
    r.onDroppedPending();
    r.onDroppedPending();
    r.onDroppedPending();
    r.onDroppedNoDriver();

    const auto s = r.snapshot(true, kMs);
    EXPECT_EQ(s.submitted, 1u);
    EXPECT_EQ(s.dropped_pending, 3u);
    EXPECT_EQ(s.dropped_no_driver, 1u);
    EXPECT_EQ(s.attempted(), 5u);
    EXPECT_DOUBLE_EQ(s.droppedPendingRatio(), 3.0 / 5.0);
}

TEST(CommandStatsRecorderTest, PendingAgeGrowsWhileACommandIsInFlightAndIsZeroOtherwise)
{
    CommandStatsRecorder r;

    r.onSubmit(1000 * kMs);

    EXPECT_EQ(r.snapshot(true, 1000 * kMs).pending_age_ms, 0u);
    EXPECT_EQ(r.snapshot(true, 1250 * kMs).pending_age_ms, 250u);
    EXPECT_TRUE(r.snapshot(true, 1250 * kMs).pending);

    // Not in flight: the age is not reported even though time has passed since the last submit.
    EXPECT_EQ(r.snapshot(false, 5000 * kMs).pending_age_ms, 0u);
}

TEST(CommandStatsRecorderTest, ClockGoingBackwardsNeverProducesANegativeLatency)
{
    CommandStatsRecorder r;

    r.onSubmit(10 * kMs);
    r.onCompletion(CommandCompletion::kOk, 0, 5 * kMs);

    EXPECT_EQ(r.snapshot(false, 20 * kMs).latency_last_us, 0u);
}

TEST(CommandStatsRecorderTest, HugeLatencySaturatesInsteadOfWrapping)
{
    CommandStatsRecorder r;

    r.onSubmit(0);
    r.onCompletion(CommandCompletion::kOk, 0, std::numeric_limits<std::int64_t>::max());

    EXPECT_EQ(r.snapshot(false, 0).latency_max_us, std::numeric_limits<std::uint32_t>::max());
}

}  // namespace rover_hardware_interface
