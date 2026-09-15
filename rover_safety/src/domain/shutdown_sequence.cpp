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

#include "rover_safety/domain/shutdown_sequence.hpp"

#include <stdexcept>
#include <string>

namespace rover_safety::domain
{

ShutdownSequence::ShutdownSequence(const std::chrono::duration<double> retry_backoff)
: retry_backoff_(retry_backoff)
{
    if (retry_backoff.count() < 0.0) {
        throw std::invalid_argument("Shutdown retry backoff must not be negative.");
    }
}

ShutdownRequestResult ShutdownSequence::request(const std::string & reason, const SteadyTime now)
{
    switch (state_) {
        case ShutdownState::InProgress: return ShutdownRequestResult::AlreadyInProgress;
        case ShutdownState::Succeeded: return ShutdownRequestResult::AlreadySucceeded;
        case ShutdownState::Failed:
            if (failed_at_.has_value() && now - *failed_at_ < retry_backoff_) {
                return ShutdownRequestResult::RetryBackoff;
            }
            break;
        case ShutdownState::Idle: break;
    }

    state_ = ShutdownState::InProgress;
    reason_ = reason;
    detail_.clear();
    failed_at_.reset();
    ++attempts_;
    return ShutdownRequestResult::Started;
}

void ShutdownSequence::finish(const bool succeeded, const std::string & detail, const SteadyTime now)
{
    if (state_ != ShutdownState::InProgress) {
        return;
    }

    if (succeeded) {
        state_ = ShutdownState::Succeeded;
        detail_.clear();
    } else {
        state_ = ShutdownState::Failed;
        detail_ = detail;
        failed_at_ = now;
    }
}

HealthLevel ShutdownSequence::healthLevel() const
{
    switch (state_) {
        case ShutdownState::InProgress:
        case ShutdownState::Succeeded: return HealthLevel::Warn;
        case ShutdownState::Failed: return HealthLevel::Error;
        case ShutdownState::Idle:
        default: return HealthLevel::Ok;
    }
}

const char * toString(const ShutdownState state)
{
    switch (state) {
        case ShutdownState::Idle: return "IDLE";
        case ShutdownState::InProgress: return "IN_PROGRESS";
        case ShutdownState::Succeeded: return "SUCCEEDED";
        case ShutdownState::Failed: return "FAILED";
    }
    return "UNKNOWN";
}

const char * toString(const ShutdownRequestResult result)
{
    switch (result) {
        case ShutdownRequestResult::Started: return "Shutdown started.";
        case ShutdownRequestResult::AlreadyInProgress: return "Shutdown already in progress.";
        case ShutdownRequestResult::AlreadySucceeded: return "Shutdown already requested; powering off.";
        case ShutdownRequestResult::RetryBackoff:
            return "Previous shutdown failed; retry refused until the retry backoff elapses.";
    }
    return "Unknown shutdown request result.";
}

}  // namespace rover_safety::domain
