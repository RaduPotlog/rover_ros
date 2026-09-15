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

#ifndef ROVER_SAFETY_DOMAIN_SHUTDOWN_SEQUENCE_HPP_
#define ROVER_SAFETY_DOMAIN_SHUTDOWN_SEQUENCE_HPP_

#include <chrono>
#include <optional>
#include <string>

#include "rover_safety/domain/safety_health.hpp"

namespace rover_safety::domain
{

enum class ShutdownState
{
    Idle,
    /** The shutdown tree is running (E-Stop, remote hosts, local power-off). */
    InProgress,
    /** The power-off request was accepted; the computer is going down. Terminal. */
    Succeeded,
    /** The power-off request failed; a new request may retry after the backoff. */
    Failed,
};

enum class ShutdownRequestResult
{
    Started,
    AlreadyInProgress,
    AlreadySucceeded,
    RetryBackoff,
};

/**
 * Tracks one shutdown of the ROS controller, whichever trigger started it (safety tree, service).
 *
 * Requests are idempotent while a shutdown is running or after it succeeded. After a failure a new
 * request retries, but not before `retry_backoff` elapsed, so a trigger that keeps firing (e.g. a
 * persistent battery verdict) cannot restart a failing power-off on every tick.
 */
class ShutdownSequence
{
public:
    using SteadyTime = std::chrono::steady_clock::time_point;

    /** Throws std::invalid_argument when retry_backoff is negative. */
    explicit ShutdownSequence(std::chrono::duration<double> retry_backoff);

    ShutdownRequestResult request(const std::string & reason, SteadyTime now);

    /** Ends a running shutdown; ignored unless the state is InProgress. */
    void finish(bool succeeded, const std::string & detail, SteadyTime now);

    ShutdownState state() const { return state_; }
    bool inProgress() const { return state_ == ShutdownState::InProgress; }
    /** Reason of the last started shutdown; empty while Idle. */
    const std::string & reason() const { return reason_; }
    /** Failure detail of the last finished shutdown; empty unless Failed. */
    const std::string & detail() const { return detail_; }
    unsigned attempts() const { return attempts_; }

    /** Idle -> Ok; InProgress and Succeeded -> Warn (the robot is going down); Failed -> Error. */
    HealthLevel healthLevel() const;

private:
    std::chrono::duration<double> retry_backoff_;
    ShutdownState state_{ShutdownState::Idle};
    std::string reason_;
    std::string detail_;
    std::optional<SteadyTime> failed_at_;
    unsigned attempts_{0};
};

const char * toString(ShutdownState state);
const char * toString(ShutdownRequestResult result);

}  // namespace rover_safety::domain

#endif  // ROVER_SAFETY_DOMAIN_SHUTDOWN_SEQUENCE_HPP_
