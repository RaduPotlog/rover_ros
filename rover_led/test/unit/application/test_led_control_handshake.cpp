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

#include <cstdint>

#include "gtest/gtest.h"

#include "rover_led/application/led_control_handshake.hpp"

using rover_led::LedControlHandshake;
using rover_led::LedControlReplyAction;
using rover_led::LedControlTickAction;

namespace
{

constexpr std::int64_t kTimeoutNs = LedControlHandshake::kResponseTimeoutNs;

// Sends an enable request at `now_ns` and ticks at the last moment it is still awaited; any
// later tick finds it timed out.
void sendAndTimeOut(LedControlHandshake & handshake, const std::int64_t now_ns)
{
    ASSERT_EQ(handshake.onTick(now_ns).action, LedControlTickAction::kSendRequest);
    handshake.onRequestSent(now_ns);
    ASSERT_EQ(handshake.onTick(now_ns + kTimeoutNs).action, LedControlTickAction::kWait);
}

}  // namespace

TEST(LedControlHandshakeTest, StartsWithoutControl)
{
    LedControlHandshake handshake;

    EXPECT_FALSE(handshake.isGranted());
    EXPECT_FALSE(handshake.hasFailed());
    EXPECT_EQ(handshake.getEpoch(), 0u);
}

TEST(LedControlHandshakeTest, ActivationWithoutHandshakeGrantsControl)
{
    LedControlHandshake handshake;
    handshake.activate(false);

    EXPECT_TRUE(handshake.isGranted());
    EXPECT_EQ(handshake.getEpoch(), 1u);
}

TEST(LedControlHandshakeTest, ActivationWithHandshakeRequestsOnTheFirstTick)
{
    LedControlHandshake handshake;
    handshake.activate(true);

    EXPECT_FALSE(handshake.isGranted());

    const auto decision = handshake.onTick(0);
    EXPECT_EQ(decision.action, LedControlTickAction::kSendRequest);
    EXPECT_FALSE(decision.response_timed_out);
}

TEST(LedControlHandshakeTest, WaitsForTheReplyUntilTheTimeoutHasPassed)
{
    LedControlHandshake handshake;
    handshake.activate(true);
    ASSERT_EQ(handshake.onTick(0).action, LedControlTickAction::kSendRequest);
    handshake.onRequestSent(0);

    const auto at_timeout = handshake.onTick(kTimeoutNs);
    EXPECT_EQ(at_timeout.action, LedControlTickAction::kWait);
    EXPECT_FALSE(at_timeout.response_timed_out);

    const auto after_timeout = handshake.onTick(kTimeoutNs + 1);
    EXPECT_TRUE(after_timeout.response_timed_out);
    EXPECT_EQ(after_timeout.action, LedControlTickAction::kSendRequest);
}

TEST(LedControlHandshakeTest, GrantStopsTheRetries)
{
    LedControlHandshake handshake;
    handshake.activate(true);
    ASSERT_EQ(handshake.onTick(0).action, LedControlTickAction::kSendRequest);
    handshake.onRequestSent(0);

    EXPECT_EQ(handshake.onReply(handshake.getEpoch(), true, true), LedControlReplyAction::kGranted);
    EXPECT_TRUE(handshake.isGranted());

    const auto decision = handshake.onTick(1);
    EXPECT_EQ(decision.action, LedControlTickAction::kStopTimer);
    EXPECT_FALSE(decision.response_timed_out);
}

TEST(LedControlHandshakeTest, RefusalIsRetriedOnTheNextTickWithoutWaiting)
{
    LedControlHandshake handshake;
    handshake.activate(true);
    ASSERT_EQ(handshake.onTick(0).action, LedControlTickAction::kSendRequest);
    handshake.onRequestSent(0);

    EXPECT_EQ(handshake.onReply(handshake.getEpoch(), true, false), LedControlReplyAction::kRefused);
    EXPECT_FALSE(handshake.isGranted());

    // Well within the response timeout.
    const auto decision = handshake.onTick(1);
    EXPECT_EQ(decision.action, LedControlTickAction::kSendRequest);
    EXPECT_FALSE(decision.response_timed_out);
}

TEST(LedControlHandshakeTest, GivesUpOnceEveryAttemptTimedOut)
{
    LedControlHandshake handshake;
    handshake.activate(true);

    std::int64_t now_ns = 0;
    ASSERT_NO_FATAL_FAILURE(sendAndTimeOut(handshake, now_ns));

    for (unsigned attempt = 2; attempt <= LedControlHandshake::kMaxAttempts; ++attempt) {
        now_ns += kTimeoutNs + 1;
        const auto decision = handshake.onTick(now_ns);
        ASSERT_TRUE(decision.response_timed_out);
        ASSERT_EQ(decision.action, LedControlTickAction::kSendRequest);
        handshake.onRequestSent(now_ns);
    }

    EXPECT_FALSE(handshake.hasFailed());

    now_ns += kTimeoutNs + 1;
    const auto decision = handshake.onTick(now_ns);
    EXPECT_TRUE(decision.response_timed_out);
    EXPECT_EQ(decision.action, LedControlTickAction::kGiveUp);
    EXPECT_TRUE(handshake.hasFailed());
    EXPECT_FALSE(handshake.isGranted());

    EXPECT_EQ(handshake.onTick(now_ns + 1).action, LedControlTickAction::kStopTimer);
}

TEST(LedControlHandshakeTest, UnavailableServiceUsesUpTheAttemptsWithoutWaiting)
{
    LedControlHandshake handshake;
    handshake.activate(true);

    // The service is not ready, so onRequestSent() is never called.
    for (unsigned attempt = 1; attempt <= LedControlHandshake::kMaxAttempts; ++attempt) {
        const auto decision = handshake.onTick(attempt);
        EXPECT_EQ(decision.action, LedControlTickAction::kSendRequest);
        EXPECT_FALSE(decision.response_timed_out);
    }

    const auto decision = handshake.onTick(LedControlHandshake::kMaxAttempts + 1);
    EXPECT_EQ(decision.action, LedControlTickAction::kGiveUp);
    EXPECT_FALSE(decision.response_timed_out);
    EXPECT_TRUE(handshake.hasFailed());
}

TEST(LedControlHandshakeTest, GrantOfAnEarlierEpochIsHandedBackNotAdopted)
{
    LedControlHandshake handshake;
    handshake.activate(true);
    ASSERT_EQ(handshake.onTick(0).action, LedControlTickAction::kSendRequest);
    handshake.onRequestSent(0);
    const auto request_epoch = handshake.getEpoch();

    // The grant only arrives once the node is inactive.
    handshake.deactivate();
    EXPECT_EQ(handshake.onReply(request_epoch, true, true), LedControlReplyAction::kReleaseStaleGrant);
    EXPECT_FALSE(handshake.isGranted());

    // Nor after a new activation.
    handshake.activate(true);
    EXPECT_EQ(handshake.onReply(request_epoch, true, true), LedControlReplyAction::kReleaseStaleGrant);
    EXPECT_FALSE(handshake.isGranted());
}

TEST(LedControlHandshakeTest, OtherRepliesOfAnEarlierEpochAreIgnored)
{
    LedControlHandshake handshake;
    handshake.activate(true);
    const auto stale_epoch = handshake.getEpoch();
    handshake.activate(true);

    EXPECT_EQ(handshake.onReply(stale_epoch, true, false), LedControlReplyAction::kIgnore);
    EXPECT_EQ(handshake.onReply(stale_epoch, false, true), LedControlReplyAction::kIgnore);
    EXPECT_EQ(handshake.onReply(stale_epoch, false, false), LedControlReplyAction::kIgnore);
    EXPECT_FALSE(handshake.isGranted());
}

TEST(LedControlHandshakeTest, ConfirmedReleaseIsReportedAsRevoked)
{
    LedControlHandshake handshake;
    handshake.activate(false);

    // The release is sent in the epoch deactivate() starts.
    ASSERT_TRUE(handshake.deactivate());
    EXPECT_EQ(handshake.onReply(handshake.getEpoch(), false, true), LedControlReplyAction::kRevoked);
    EXPECT_FALSE(handshake.isGranted());
}

TEST(LedControlHandshakeTest, DeactivationReportsWhetherControlWasGranted)
{
    LedControlHandshake handshake;
    handshake.activate(false);
    const auto epoch = handshake.getEpoch();

    EXPECT_TRUE(handshake.deactivate());
    EXPECT_FALSE(handshake.isGranted());
    EXPECT_EQ(handshake.getEpoch(), epoch + 1);

    EXPECT_FALSE(handshake.deactivate());
    EXPECT_EQ(handshake.getEpoch(), epoch + 2);
}

TEST(LedControlHandshakeTest, InvalidateOnlyStartsANewEpoch)
{
    LedControlHandshake handshake;
    handshake.activate(false);
    const auto epoch = handshake.getEpoch();

    handshake.invalidate();

    EXPECT_TRUE(handshake.isGranted());
    EXPECT_FALSE(handshake.hasFailed());
    EXPECT_EQ(handshake.getEpoch(), epoch + 1);
}

TEST(LedControlHandshakeTest, ActivationForgetsAttemptsAndFailure)
{
    LedControlHandshake handshake;
    handshake.activate(true);

    for (unsigned attempt = 1; attempt <= LedControlHandshake::kMaxAttempts; ++attempt) {
        ASSERT_EQ(handshake.onTick(attempt).action, LedControlTickAction::kSendRequest);
    }
    ASSERT_EQ(handshake.onTick(LedControlHandshake::kMaxAttempts + 1).action, LedControlTickAction::kGiveUp);

    // Deactivation keeps the failure; the next activation starts over.
    handshake.deactivate();
    EXPECT_TRUE(handshake.hasFailed());

    handshake.activate(true);
    EXPECT_FALSE(handshake.hasFailed());

    for (unsigned attempt = 1; attempt <= LedControlHandshake::kMaxAttempts; ++attempt) {
        EXPECT_EQ(handshake.onTick(attempt).action, LedControlTickAction::kSendRequest);
    }
    EXPECT_EQ(handshake.onTick(LedControlHandshake::kMaxAttempts + 1).action, LedControlTickAction::kGiveUp);
}

TEST(LedControlHandshakeTest, LateGrantAfterGivingUpIsStillAdopted)
{
    LedControlHandshake handshake;
    handshake.activate(true);

    std::int64_t now_ns = 0;
    ASSERT_NO_FATAL_FAILURE(sendAndTimeOut(handshake, now_ns));

    for (unsigned attempt = 2; attempt <= LedControlHandshake::kMaxAttempts; ++attempt) {
        now_ns += kTimeoutNs + 1;
        ASSERT_EQ(handshake.onTick(now_ns).action, LedControlTickAction::kSendRequest);
        handshake.onRequestSent(now_ns);
    }

    now_ns += kTimeoutNs + 1;
    ASSERT_EQ(handshake.onTick(now_ns).action, LedControlTickAction::kGiveUp);

    // A reply to one of the timed-out requests of this activation.
    EXPECT_EQ(handshake.onReply(handshake.getEpoch(), true, true), LedControlReplyAction::kGranted);
    EXPECT_TRUE(handshake.isGranted());
    EXPECT_TRUE(handshake.hasFailed());
}

TEST(LedControlHandshakeTest, AnyReplyOfTheEpochEndsTheWaitForTheLatestRequest)
{
    LedControlHandshake handshake;
    handshake.activate(true);

    // The first request times out, the second is sent...
    ASSERT_NO_FATAL_FAILURE(sendAndTimeOut(handshake, 0));
    ASSERT_EQ(handshake.onTick(kTimeoutNs + 1).action, LedControlTickAction::kSendRequest);
    handshake.onRequestSent(kTimeoutNs + 1);
    ASSERT_EQ(handshake.onTick(kTimeoutNs + 2).action, LedControlTickAction::kWait);

    // ...and the late refusal of the first one ends the wait for the second.
    EXPECT_EQ(handshake.onReply(handshake.getEpoch(), true, false), LedControlReplyAction::kRefused);

    const auto decision = handshake.onTick(kTimeoutNs + 3);
    EXPECT_EQ(decision.action, LedControlTickAction::kSendRequest);
    EXPECT_FALSE(decision.response_timed_out);
}
