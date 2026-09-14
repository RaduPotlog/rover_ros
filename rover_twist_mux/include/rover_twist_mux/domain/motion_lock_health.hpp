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
 * @brief Grades the motion lock.
 * @param flags          Last received safety-IO state, or nullopt when nothing arrived yet.
 * @param gpio_age_s     Seconds since `flags` was received. Ignored when `flags` is nullopt.
 * @param gpio_timeout_s How long `flags` stays trusted.
 */
MotionLockHealth evaluateMotionLockHealth(
    const std::optional<SafetyIoFlags> & flags,
    double gpio_age_s,
    double gpio_timeout_s,
    const MotionLockPolicy & policy);

}  // namespace rover_twist_mux::domain

#endif  // ROVER_TWIST_MUX_DOMAIN_MOTION_LOCK_HEALTH_HPP_
