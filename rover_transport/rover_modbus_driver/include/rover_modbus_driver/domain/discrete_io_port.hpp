// Copyright 2026 Mechatronics Academy
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

#ifndef ROVER_MODBUS_DRIVER_DOMAIN_DISCRETE_IO_PORT_HPP_
#define ROVER_MODBUS_DRIVER_DOMAIN_DISCRETE_IO_PORT_HPP_

#include <cstdint>

#include "rover_modbus_driver/domain/contact_coil_types.hpp"

namespace rover::transport::modbus
{

// Sentinel returned by the read methods when the device answered with something that is
// not a coil value. Carried over from the 255U literal the previous implementation used.
//
// In practice this is unreachable: a short reply throws MB::ModbusException out of
// registerValues() before it gets here, and the codec coerces discrete-read values to
// coils. Genuine read failures arrive as exceptions, not as this value.
constexpr uint16_t kDiscreteReadUnavailable = 255U;

// The discrete-IO primitives a caller needs from whatever speaks Modbus to the device.
// Implemented by ModbusDiscreteIoClient and by fakes in tests, so coil-mapping and
// enable-guard logic above it can be unit-tested without a live connection.
//
// This was RoverModbusInterface in rover_hardware_interface.
class DiscreteIoPort
{

public:

    virtual ~DiscreteIoPort() = default;

    virtual uint16_t readDiscreteContact(const ContactInfo & contact) = 0;

    virtual uint16_t readDiscreteCoil(const CoilInfo & coil) = 0;

    virtual void writeDiscreteCoil(const CoilInfo & coil, const bool coil_state) = 0;
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_DOMAIN_DISCRETE_IO_PORT_HPP_
