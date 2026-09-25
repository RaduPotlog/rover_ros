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

#include "rover_led/application/led_control_handshake.hpp"

#include <cstdint>

namespace rover_led
{

void LedControlHandshake::activate(const bool handshake)
{
    epoch_++;
    pending_ = false;
    failed_ = false;
    attempts_ = 0;
    granted_ = !handshake;
}

bool LedControlHandshake::deactivate()
{
    epoch_++;
    pending_ = false;

    const bool was_granted = granted_;
    granted_ = false;

    return was_granted;
}

void LedControlHandshake::invalidate()
{
    epoch_++;
}

LedControlTickDecision LedControlHandshake::onTick(const std::int64_t now_ns)
{
    LedControlTickDecision decision;

    if (granted_ || failed_) {
        decision.action = LedControlTickAction::kStopTimer;
        return decision;
    }

    if (pending_) {
        if (now_ns - request_time_ns_ <= kResponseTimeoutNs) {
            decision.action = LedControlTickAction::kWait;
            return decision;
        }

        decision.response_timed_out = true;
        pending_ = false;
    }

    if (attempts_ >= kMaxAttempts) {
        failed_ = true;
        decision.action = LedControlTickAction::kGiveUp;
        return decision;
    }

    attempts_++;
    decision.action = LedControlTickAction::kSendRequest;

    return decision;
}

void LedControlHandshake::onRequestSent(const std::int64_t now_ns)
{
    pending_ = true;
    request_time_ns_ = now_ns;
}

LedControlReplyAction LedControlHandshake::onReply(
    const std::uint64_t epoch, const bool requested_enable, const bool success)
{
    if (epoch != epoch_) {
        return (requested_enable && success) ? LedControlReplyAction::kReleaseStaleGrant
                                             : LedControlReplyAction::kIgnore;
    }

    // Any reply of this epoch ends the wait, even one to an earlier request.
    pending_ = false;

    if (!success) {
        return LedControlReplyAction::kRefused;
    }

    if (!requested_enable) {
        return LedControlReplyAction::kRevoked;
    }

    // Adopted even after giving up: control was granted after all.
    granted_ = true;

    return LedControlReplyAction::kGranted;
}

}  // namespace rover_led
