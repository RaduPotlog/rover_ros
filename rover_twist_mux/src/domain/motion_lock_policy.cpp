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

const char * toString(MotionInhibitReason reason)
{
    switch (reason) {
        case MotionInhibitReason::HwEStopUserButton: return "hardware E-Stop button pressed";
        case MotionInhibitReason::SwEStopUserButton: return "software E-Stop triggered";
        case MotionInhibitReason::MotorDriverFault: return "motor driver fault";
        case MotionInhibitReason::EStopLatched: return "safety latch held";
        case MotionInhibitReason::MotorContactorDisengaged: return "motor contactor not engaged";
    }

    return "unknown";
}

std::vector<MotionInhibitReason> motionInhibitReasons(
    const SafetyIoFlags & flags, const MotionLockPolicy & policy)
{
    std::vector<MotionInhibitReason> reasons;

    // Active-high stop conditions: any one of them inhibits motion.
    if (policy.use_hw_e_stop_user_button && flags.hw_e_stop_user_button) {
        reasons.push_back(MotionInhibitReason::HwEStopUserButton);
    }

    if (policy.use_sw_e_stop_user_button && flags.sw_e_stop_user_button) {
        reasons.push_back(MotionInhibitReason::SwEStopUserButton);
    }

    if (policy.use_sw_e_stop_motor_driver_fault && flags.sw_e_stop_motor_driver_fault) {
        reasons.push_back(MotionInhibitReason::MotorDriverFault);
    }

    if (policy.use_sw_e_stop_latch_status && flags.sw_e_stop_latch_status) {
        reasons.push_back(MotionInhibitReason::EStopLatched);
    }

    // Inverted sense: the contactor being open means the drive is dead, so commanding motion is
    // meaningless at best. Checked last because it is the only opt-in condition.
    if (policy.require_motor_contactor_engaged && !flags.motor_contactor_engaged) {
        reasons.push_back(MotionInhibitReason::MotorContactorDisengaged);
    }

    return reasons;
}

bool isMotionInhibited(const SafetyIoFlags & flags, const MotionLockPolicy & policy)
{
    return !motionInhibitReasons(flags, policy).empty();
}

}  // namespace rover_twist_mux::domain
