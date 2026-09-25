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
#include "rover_hardware_interface/domain/safety_link_health.hpp"

namespace rover_hardware_interface
{

// Port: the GPIO/safety-controller primitives RoverSystem needs to start the controller, drive
// the aux outputs, poll its IO state and report link health, without depending on the concrete
// Modbus-backed RoverSafetyController directly. It deliberately has no E-Stop coil writes: those
// belong to EmergencyStopIoPort, behind EmergencyStop's rules. Implemented by
// RoverSafetyControllerGpioAdapter (see
// rover_safety_controller/rover_safety_controller_gpio_adapter.hpp), mirroring how
// EmergencyStopIoPort/RoverSafetyControllerEStopIo isolate EmergencyStop from the same concrete
// type.
class RoverGpioPort
{

public:

    virtual ~RoverGpioPort() = default;

    virtual void start() = 0;

    // Drives general-purpose output GPIO_AUX_OUT_<index> (index < kAuxOutputCount). NOT RT-safe:
    // blocks for a Modbus round-trip and throws on failure, an out-of-range index, or before
    // start(). Call from a service thread only, never from read()/write().
    virtual void setAuxOutput(const unsigned index, const bool state) = 0;

    // Non-blocking; returns a reference to a cache owned by the implementation. Safe to call
    // from the RT thread (see RoverSafetyController::queryControlInterfaceIOStates()).
    virtual const std::unordered_map<RoverControllerGpio, bool> & queryControlInterfaceIOStates() = 0;

    // Health of the link and of the threads servicing it, for the diagnostics task. Non-blocking.
    virtual SafetyLinkHealth linkHealth() const = 0;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_GPIO_PORT_HPP_
