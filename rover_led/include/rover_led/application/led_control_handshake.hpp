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

#ifndef ROVER_LED_APPLICATION_LED_CONTROL_HANDSHAKE_HPP_
#define ROVER_LED_APPLICATION_LED_CONTROL_HANDSHAKE_HPP_

#include <cstdint>

namespace rover_led
{

enum class LedControlTickAction
{
    kWait,         // a request is pending within its response timeout
    kStopTimer,    // control was granted or given up on
    kSendRequest,  // send an enable request (the attempt is used even if it can't be sent)
    kGiveUp,       // every attempt used: stop the timer, frames stay ignored
};

struct LedControlTickDecision
{
    // The pending request got no reply in time.
    bool response_timed_out = false;
    LedControlTickAction action = LedControlTickAction::kWait;
};

enum class LedControlReplyAction
{
    kIgnore,             // a refusal or release of an earlier epoch
    kReleaseStaleGrant,  // control granted to an earlier epoch: hand it back
    kRefused,            // retried on the next tick, without waiting for the timeout
    kRevoked,            // a release was confirmed
    kGranted,
};

// Obtaining exclusive LED control for one activation: up to kMaxAttempts requests, each given
// kResponseTimeoutNs to be answered; replies to requests of an earlier epoch (activation,
// deactivation, cleanup) are never adopted. Bookkeeping only: the caller sends the requests
// and passes in the clock and the replies. Times are nanoseconds on one clock.
class LedControlHandshake
{

public:

    static constexpr unsigned kMaxAttempts = 3;
    static constexpr std::int64_t kResponseTimeoutNs = 3'000'000'000;

    // Starts an epoch with no attempts used; without the handshake control is granted at once.
    void activate(const bool handshake);

    // Starts an epoch and drops control; returns whether it was granted (and must be released).
    bool deactivate();

    // Starts an epoch, so replies still in flight are stale.
    void invalidate();

    LedControlTickDecision onTick(const std::int64_t now_ns);

    // Call when kSendRequest could be sent; the reply is then awaited.
    void onRequestSent(const std::int64_t now_ns);

    // `epoch` is the one the request was sent in, `requested_enable` its SetBool data.
    LedControlReplyAction onReply(const std::uint64_t epoch, const bool requested_enable, const bool success);

    std::uint64_t getEpoch() const
    {
        return epoch_;
    }

    bool isGranted() const
    {
        return granted_;
    }

    bool hasFailed() const
    {
        return failed_;
    }

private:

    bool granted_ = false;
    bool pending_ = false;
    bool failed_ = false;
    unsigned attempts_ = 0;
    std::uint64_t epoch_ = 0;
    std::int64_t request_time_ns_ = 0;
};

}  // namespace rover_led

#endif  // ROVER_LED_APPLICATION_LED_CONTROL_HANDSHAKE_HPP_
