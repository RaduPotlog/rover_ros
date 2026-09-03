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

#include "rover_hardware_interface/rover_controller/rover_controller_gpio_adapter.hpp"

#include <utility>

namespace rover_hardware_interface
{

RoverControllerGpioAdapter::RoverControllerGpioAdapter(
    std::shared_ptr<RoverController> rover_controller)
: rover_controller_(std::move(rover_controller))
{

}

void RoverControllerGpioAdapter::start()
{
    rover_controller_->start();
}

void RoverControllerGpioAdapter::eStopUserBtnTrigger(const bool state)
{
    rover_controller_->eStopUserBtnTrigger(state);
}

void RoverControllerGpioAdapter::eStopMotorDriverFaultTrigger(const bool state)
{
    rover_controller_->eStopMotorDriverFaultTrigger(state);
}

const std::unordered_map<RoverControllerGpio, bool> &
RoverControllerGpioAdapter::queryControlInterfaceIOStates()
{
    return rover_controller_->queryControlInterfaceIOStates();
}

}  // namespace rover_hardware_interface
