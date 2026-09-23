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
#include <vector>

#include "rover_twist_mux/domain/command_freshness_filter.hpp"

using namespace rover_twist_mux::domain;  // NOLINT

namespace
{

constexpr double kPeriod = 0.1;         // the Driver UI's 10 Hz stream
constexpr double kEpoch = 1.8e9;        // a realistic wall-clock epoch, to exercise double precision
constexpr double kTransport = 0.02;     // best-case one-way delay

// Deterministic jitter in [0, amplitude): no RNG, so a failure reproduces exactly.
double jitter(int i, double amplitude)
{
    return amplitude * (0.5 + 0.5 * std::sin(i * 1.7));
}

class CommandFreshnessFilterTest : public ::testing::Test
{
protected:
    // One command sent at `sent` (rover wall clock) and received `delay` later, stamped by a sender
    // whose clock reads `clock_offset` ahead of the rover's.
    FreshnessVerdict deliver(double sent, double delay, double clock_offset = 0.0)
    {
        return filter_.accept(kEpoch + sent + clock_offset, kEpoch + sent + delay);
    }

    // A steady stream from `start` for `duration`; returns how many were accepted.
    int stream(double start, double duration, double clock_offset = 0.0, double jitter_amp = 0.0)
    {
        int accepted = 0;
        const int n = static_cast<int>(std::lround(duration / kPeriod));
        for (int i = 0; i < n; ++i) {
            const double delay = kTransport + jitter(i, jitter_amp);
            accepted += isAccepted(deliver(start + i * kPeriod, delay, clock_offset)) ? 1 : 0;
        }
        return accepted;
    }

    CommandFreshnessFilter filter_{CommandFreshnessConfig{}};
};

TEST_F(CommandFreshnessFilterTest, FirstStampedCommandIsAccepted)
{
    EXPECT_EQ(deliver(0.0, kTransport), FreshnessVerdict::Fresh);
    ASSERT_TRUE(filter_.stats().baseline_s.has_value());
    EXPECT_NEAR(*filter_.stats().baseline_s, kTransport, 1e-6);
}

// The sender's clock being seconds away from the rover's must not matter - that is the reason
// the filter works on relative delay.
TEST_F(CommandFreshnessFilterTest, ConstantClockOffsetIsAccepted)
{
    EXPECT_EQ(stream(0.0, 10.0, 5.0), 100);

    CommandFreshnessFilter behind{CommandFreshnessConfig{}};
    filter_ = behind;
    EXPECT_EQ(stream(0.0, 10.0, -5.0), 100);
}

TEST_F(CommandFreshnessFilterTest, JitterWithinMaxDelayIsAccepted)
{
    EXPECT_EQ(stream(0.0, 30.0, 0.0, 0.25), 300);
    EXPECT_EQ(filter_.stats().rejected, 0u);
}

TEST_F(CommandFreshnessFilterTest, UnstampedCommandIsDropped)
{
    stream(0.0, 1.0);
    EXPECT_EQ(filter_.accept(0.0, kEpoch + 1.0), FreshnessVerdict::Unstamped);
    EXPECT_EQ(filter_.stats().rejected, 1u);
}

// The case the filter exists for: a 2 s Wi-Fi stall. The websocket holds 20 commands and then
// delivers them together; only those sent within max_delay of delivery may drive.
TEST_F(CommandFreshnessFilterTest, StallBurstIsDroppedAndItsFreshTailPasses)
{
    ASSERT_EQ(stream(0.0, 5.0), 50);

    const double stall_start = 5.0;
    const double delivered_at = 7.0;
    std::vector<FreshnessVerdict> verdicts;
    for (int i = 0; i < 20; ++i) {
        const double sent = stall_start + i * kPeriod;
        verdicts.push_back(filter_.accept(kEpoch + sent, kEpoch + delivered_at + i * 1e-4));
    }

    // Sent at 5.0 .. 6.9, all received at ~7.0: late by 2.0 .. 0.1 s beyond transport.
    int dropped = 0;
    for (const auto verdict : verdicts) {
        dropped += verdict == FreshnessVerdict::Stale ? 1 : 0;
    }
    EXPECT_GE(dropped, 17);
    EXPECT_EQ(verdicts.front(), FreshnessVerdict::Stale);
    EXPECT_TRUE(isAccepted(verdicts.back()));

    // Normal delivery resumes and passes.
    EXPECT_EQ(stream(7.0, 3.0), 30);
    EXPECT_EQ(filter_.stats().resyncs, 0u);
}

// Silence must not loosen the baseline: a burst after a long stall is judged against the
// baseline from before it.
TEST_F(CommandFreshnessFilterTest, BurstAfterAStallLongerThanResyncTimeIsStillDropped)
{
    ASSERT_EQ(stream(0.0, 5.0), 50);

    const double delivered_at = 35.0;
    int accepted = 0;
    for (int i = 0; i < 300; ++i) {
        const double sent = 5.0 + i * kPeriod;
        accepted += isAccepted(filter_.accept(kEpoch + sent, kEpoch + delivered_at + i * 1e-4));
    }

    // Only the last ~3 (sent within max_delay of delivery) may pass.
    EXPECT_LE(accepted, 4);
    EXPECT_EQ(filter_.stats().resyncs, 0u);
}

// NTP-disciplined clocks drift by far less than 50 ppm; an hour of it must never be dropped.
TEST_F(CommandFreshnessFilterTest, FollowsClockDrift)
{
    for (const double ppm : {50.0, -50.0}) {
        filter_ = CommandFreshnessFilter{CommandFreshnessConfig{}};
        int accepted = 0;
        const int n = 36000;  // one hour at 10 Hz
        for (int i = 0; i < n; ++i) {
            const double sent = i * kPeriod;
            const double offset = sent * ppm * 1e-6;
            accepted += isAccepted(deliver(sent, kTransport + jitter(i, 0.05), offset)) ? 1 : 0;
        }
        EXPECT_EQ(accepted, n) << ppm << " ppm";
    }
}

// A backlog building up while commands flow (latency growing 20 ms per second) must eventually
// be dropped, and must not be able to resync itself.
TEST_F(CommandFreshnessFilterTest, GrowingBacklogIsDroppedAndCannotResync)
{
    ASSERT_EQ(stream(0.0, 5.0), 50);

    int last_accepted = -1;
    for (int i = 0; i < 600; ++i) {  // 60 s
        const double sent = 5.0 + i * kPeriod;
        const double backlog = 0.02 * i * kPeriod;
        if (isAccepted(deliver(sent, kTransport + backlog))) {
            last_accepted = i;
        }
    }

    // 0.3 s of excess at (20 - 1) ms/s is ~16 s in.
    EXPECT_GT(last_accepted, 100);
    EXPECT_LT(last_accepted, 200);
    EXPECT_EQ(filter_.stats().resyncs, 0u);
}

// The browser's clock stepping back 5 s between two Manual sessions makes every command look
// 5 s late. After resync_time of steady commands the filter re-baselines.
TEST_F(CommandFreshnessFilterTest, BackwardClockStepAfterSilenceResyncs)
{
    ASSERT_EQ(stream(0.0, 5.0), 50);

    // 3 s of silence, then the next session with the clock 5 s behind.
    const double restart = 8.0;
    std::vector<FreshnessVerdict> verdicts;
    for (int i = 0; i < 50; ++i) {
        verdicts.push_back(deliver(restart + i * kPeriod, kTransport + jitter(i, 0.05), -5.0));
    }

    int first_accepted = -1;
    for (int i = 0; i < static_cast<int>(verdicts.size()); ++i) {
        if (isAccepted(verdicts[i])) {
            first_accepted = i;
            break;
        }
    }

    // Dropped for resync_time (2 s = 20 commands), then passing again.
    EXPECT_EQ(first_accepted, 20);
    EXPECT_EQ(verdicts[20], FreshnessVerdict::Resynced);
    for (int i = 21; i < 50; ++i) {
        EXPECT_EQ(verdicts[i], FreshnessVerdict::Fresh) << i;
    }
    EXPECT_EQ(filter_.stats().resyncs, 1u);
}

// The same step while commands keep flowing has no preceding silence, so it does not resync:
// indistinguishable from a backlog, and dropping is the safe direction. It recovers after the
// operator lets go and the next session starts.
TEST_F(CommandFreshnessFilterTest, BackwardClockStepMidStreamWaitsForSilenceToResync)
{
    ASSERT_EQ(stream(0.0, 5.0), 50);

    EXPECT_EQ(stream(5.0, 5.0, -5.0), 0);
    EXPECT_EQ(filter_.stats().resyncs, 0u);

    // Silence, then a new session: dropped for resync_time (20 commands), then passing.
    const int accepted = stream(12.0, 5.0, -5.0);
    EXPECT_GE(accepted, 29);
    EXPECT_LE(accepted, 30);
    EXPECT_EQ(filter_.stats().resyncs, 1u);
}

// A stale burst arrives within milliseconds: it never spans resync_time, so it cannot resync
// however many commands it holds.
TEST_F(CommandFreshnessFilterTest, StaleBurstNeverResyncs)
{
    ASSERT_EQ(stream(0.0, 5.0), 50);

    for (int i = 0; i < 1000; ++i) {
        filter_.accept(kEpoch + 5.0 + i * 0.001, kEpoch + 60.0 + i * 1e-5);
    }

    EXPECT_EQ(filter_.stats().resyncs, 0u);
}

TEST_F(CommandFreshnessFilterTest, LowerLatencyLowersTheBaselineAtOnce)
{
    stream(0.0, 2.0, 0.0, 0.0);
    ASSERT_EQ(deliver(2.0, 0.005), FreshnessVerdict::Fresh);
    EXPECT_NEAR(*filter_.stats().baseline_s, 0.005, 1e-6);
}

}  // namespace
