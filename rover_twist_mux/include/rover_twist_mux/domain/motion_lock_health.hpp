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


#ifndef ROVER_TWIST_MUX_DOMAIN_MOTION_LOCK_HEALTH_HPP_
#define ROVER_TWIST_MUX_DOMAIN_MOTION_LOCK_HEALTH_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rover_twist_mux/domain/motion_lock_policy.hpp"
#include "rover_twist_mux/domain/safety_io_flags.hpp"

namespace rover_twist_mux::domain
{

/** @brief Ordered by severity. */
enum class HealthLevel
{
    Ok,
    Warn,
    Error,
};

/**
 * @brief The motion lock decision together with why it was made.
 * @details The node publishes `locked` and feeds the whole report to diagnostics, so the lock
 *          topic and the diagnostic can never disagree.
 */
struct MotionLockHealth
{
    /// Error: the lock cannot trust its input. Warn: a stop condition holds the lock. Ok: unlocked.
    HealthLevel level{HealthLevel::Error};
    bool locked{true};
    std::string message;
    std::vector<MotionInhibitReason> reasons;
};

/**
 * @brief How long ago each of the two safety messages was received, in seconds.
 * @details Passed apart rather than pre-combined, so the rule that combines them (the older
 *          one counts) lives in evaluateMotionLockHealth() next to the timeout it is judged
 *          against, where the unit tests reach it.
 */
struct SafetyStateAges
{
    double safety_status_s{0.0};
    double safety_command_echo_s{0.0};
};

/**
 * @brief Grades the motion lock.
 * @param flags          Last received safety-IO state, or nullopt when either safety topic has
 *                       not arrived yet. Both are required: a half-populated view would silently
 *                       treat the missing half as "no stop".
 * @param ages           Seconds since each safety message was received. The OLDER of the two is
 *                       judged against `gpio_timeout_s`, so neither topic going quiet on its own
 *                       can hide behind the other still arriving. Ignored when `flags` is nullopt.
 * @param gpio_timeout_s How long `flags` stays trusted.
 * @param link_healthy   SafetyStatus.link_healthy: whether the hardware interface's link to the
 *                       safety PLC is up. False means the flags are last-known-good rather than
 *                       current - the messages still arrive, so staleness alone will not catch it.
 */
MotionLockHealth evaluateMotionLockHealth(
    const std::optional<SafetyIoFlags> & flags,
    const SafetyStateAges & ages,
    double gpio_timeout_s,
    const MotionLockPolicy & policy,
    bool link_healthy = true);

}  // namespace rover_twist_mux::domain

#endif  // ROVER_TWIST_MUX_DOMAIN_MOTION_LOCK_HEALTH_HPP_
