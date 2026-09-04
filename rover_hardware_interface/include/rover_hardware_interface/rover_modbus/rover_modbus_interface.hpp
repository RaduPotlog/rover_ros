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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_MODBUS_ROVER_MODBUS_INTERFACE_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_MODBUS_ROVER_MODBUS_INTERFACE_HPP_

#include <cstdint>

#include "rover_hardware_interface/rover_modbus/modbus_types.hpp"

namespace rover_hardware_interface
{

// Interface: the discrete-IO primitives ContactCoilHandler/RoverSafetyController need from
// whatever actually speaks Modbus to the safety-relay board. Implemented by the real, TCP-backed
// RoverModbus and by a fake in tests, so ContactCoilHandler's/RoverSafetyController's coil-mapping
// and enabled-guard logic can be unit-tested without a live Modbus TCP connection. This mirrors
// RoverDriverInterface's role for PhidgetRoverDriver - a plain interface-extraction seam within
// the infrastructure layer, not a domain port (ContactCoilHandler/RoverSafetyController are
// themselves infrastructure, so this has no domain-purity implications).
class RoverModbusInterface
{

public:

    virtual ~RoverModbusInterface() = default;

    virtual uint16_t readDiscreteContact(const ContactInfo & contact) = 0;

    virtual uint16_t readDiscreteCoil(const CoilInfo & coil) = 0;

    virtual void writeDiscreteCoil(const CoilInfo & coil, const bool coil_state) = 0;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_MODBUS_ROVER_MODBUS_INTERFACE_HPP_
