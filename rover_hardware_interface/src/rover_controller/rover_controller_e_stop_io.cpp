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

#include "rover_hardware_interface/rover_controller/rover_controller_e_stop_io.hpp"

#include <utility>

namespace rover_hardware_interface
{

RoverControllerEStopIo::RoverControllerEStopIo(std::shared_ptr<RoverController> rover_controller)
: rover_controller_(std::move(rover_controller))
{

}

bool RoverControllerEStopIo::isUserButtonActive()
{
    return rover_controller_->isPinActive(RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON);
}

bool RoverControllerEStopIo::isLatchActive()
{
    return rover_controller_->isPinActive(RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS);
}

void RoverControllerEStopIo::triggerUserButton(const bool state)
{
    rover_controller_->eStopUserBtnTrigger(state);
}

void RoverControllerEStopIo::resetLatch()
{
    rover_controller_->eStopLatchReset();
}

}  // namespace rover_hardware_interface
