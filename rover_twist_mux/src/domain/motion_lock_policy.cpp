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

#include "rover_twist_mux/domain/motion_lock_policy.hpp"

namespace rover_twist_mux::domain
{

bool isMotionInhibited(const SafetyIoFlags & flags, const MotionLockPolicy & policy)
{
    // Active-high stop conditions: any one of them inhibits motion.
    if (policy.use_hw_e_stop_user_button && flags.hw_e_stop_user_button) {
        return true;
    }

    if (policy.use_sw_e_stop_user_button && flags.sw_e_stop_user_button) {
        return true;
    }

    if (policy.use_sw_e_stop_motor_driver_fault && flags.sw_e_stop_motor_driver_fault) {
        return true;
    }

    if (policy.use_sw_e_stop_latch_status && flags.sw_e_stop_latch_status) {
        return true;
    }

    // Inverted sense: the contactor being open means the drive is dead, so commanding motion is
    // meaningless at best. Checked last because it is the only opt-in condition.
    if (policy.require_motor_contactor_engaged && !flags.motor_contactor_engaged) {
        return true;
    }

    return false;
}

}  // namespace rover_twist_mux::domain
