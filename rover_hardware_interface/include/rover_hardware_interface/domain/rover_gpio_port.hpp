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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_GPIO_PORT_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_GPIO_PORT_HPP_

#include <unordered_map>

#include "rover_hardware_interface/domain/rover_gpio_types.hpp"

namespace rover_hardware_interface
{

// Port: the GPIO/safety-controller primitives RoverSystem needs to arm the controller and poll
// its IO state, without depending on the concrete Modbus-backed RoverSafetyController directly.
// Implemented by RoverSafetyControllerGpioAdapter (see
// rover_safety_controller/rover_safety_controller_gpio_adapter.hpp), mirroring how
// EmergencyStopIoPort/RoverSafetyControllerEStopIo isolate EmergencyStop from the same concrete
// type.
class RoverGpioPort
{

public:

    virtual ~RoverGpioPort() = default;

    virtual void start() = 0;

    // SW E-STOP USER BTN - sw_e_stop_user_button
    virtual void eStopUserBtnTrigger(const bool state) = 0;

    // SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
    virtual void eStopMotorDriverFaultTrigger(const bool state) = 0;

    // Non-blocking; returns a reference to a cache owned by the implementation. Safe to call
    // from the RT thread (see RoverSafetyController::queryControlInterfaceIOStates()).
    virtual const std::unordered_map<RoverControllerGpio, bool> & queryControlInterfaceIOStates() = 0;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_GPIO_PORT_HPP_
