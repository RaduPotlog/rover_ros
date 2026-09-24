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

#ifndef ROVER_MODBUS_DRIVER_DOMAIN_MODBUS_TRANSPORT_PORT_HPP_
#define ROVER_MODBUS_DRIVER_DOMAIN_MODBUS_TRANSPORT_PORT_HPP_

#include <MB/modbusException.hpp>
#include <MB/modbusRequest.hpp>
#include <MB/modbusResponse.hpp>

namespace rover::transport::modbus
{

// The transport seam: one synchronous Modbus transaction. ModbusTcpTransport is the only
// implementation today; a serial/RTU or libmodbus-backed one would be a sibling.
//
// This was ModbusConnection in rover_hardware_interface. Note that it names MB:: types,
// which is why this package's _core library links Modbus_Core - those are pure frame
// codecs with no sockets behind them, so _core stays OS-free. Treating them as this
// package's domain vocabulary is deliberate: it exists to drive that codec. Packages above
// it (rover_hardware_interface's check_domain_purity.sh) keep MB:: out of their domain.
class ModbusTransportPort
{

public:

    ModbusTransportPort() = default;

    // Modified 2026 by Mechatronics Academy: added. This class was held by
    // std::unique_ptr<ModbusConnection> but declared no virtual destructor, so deleting
    // through the base was undefined behaviour - and in practice ~ModbusTcpConnection
    // never ran, leaking the socket fd on every teardown. close() was a documented no-op,
    // so nothing else closed it either.
    virtual ~ModbusTransportPort() = default;

    ModbusTransportPort(const ModbusTransportPort &) = delete;
    ModbusTransportPort & operator=(const ModbusTransportPort &) = delete;

    virtual MB::ModbusResponse sendRequest(const MB::ModbusRequest & req) = 0;

    virtual void close() = 0;
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_DOMAIN_MODBUS_TRANSPORT_PORT_HPP_
