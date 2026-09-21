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

#include "rover_crsf_teleop/domain/safety_io_flags.hpp"

namespace rover_crsf_teleop
{

bool isSafeToCalibrate(const SafetyIoFlags & flags)
{
    // AND, not OR: this grants a permit. See the header.
    return flags.hw_e_stop_user_button && flags.sw_e_stop_latch_status &&
           !flags.motor_contactor_engaged;
}

}  // namespace rover_crsf_teleop
