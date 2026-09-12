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

#ifndef ROVER_TWIST_MUX_DOMAIN_MOTION_LOCK_POLICY_HPP_
#define ROVER_TWIST_MUX_DOMAIN_MOTION_LOCK_POLICY_HPP_

#include "rover_twist_mux/domain/safety_io_flags.hpp"

namespace rover_twist_mux::domain
{

/**
 * @brief Which safety-IO pins contribute to the motion lock.
 * @details Each flag opts one pin into the decision. Everything enabled here is a stop condition
 *          in its ACTIVE-high sense; `require_motor_contactor_engaged` is the exception and reads
 *          the contactor's inverted sense (not engaged => inhibit).
 *
 *          `gpio_pin_sw_e_stop_latch_reset` is deliberately absent: it is a command pulse written
 *          to the safety PLC to clear the latch, not a state to be gated on. Gating on it would
 *          invert the meaning of a reset.
 */
struct MotionLockPolicy
{
    bool use_hw_e_stop_user_button = true;
    bool use_sw_e_stop_user_button = true;
    bool use_sw_e_stop_cpu_wdg_trigger = true;
    bool use_sw_e_stop_motor_driver_fault = true;
    bool use_sw_e_stop_latch_status = true;

    /// Off by default: nothing else in the stack treats the contactor as a safety predicate, and
    /// its polarity is the inverse of every other pin. Enable only once verified on hardware.
    bool require_motor_contactor_engaged = false;
};

/**
 * @brief Decides whether motion must be inhibited for the given safety-IO state.
 * @return `true` when at least one enabled stop condition is active, i.e. the rover must not be
 *         commanded to move. With no pin enabled the result is `false` (nothing to inhibit on).
 */
bool isMotionInhibited(const SafetyIoFlags & flags, const MotionLockPolicy & policy);

}  // namespace rover_twist_mux::domain

#endif  // ROVER_TWIST_MUX_DOMAIN_MOTION_LOCK_POLICY_HPP_
