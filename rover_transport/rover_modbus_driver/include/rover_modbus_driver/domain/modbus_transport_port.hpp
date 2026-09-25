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

#include "rover_modbus_driver/domain/discrete_transaction.hpp"

namespace rover::transport::modbus
{

// The transport seam: one synchronous discrete-IO transaction, in this package's terms
// (domain/discrete_transaction.hpp). ModbusTcpTransport is the only implementation today; a
// serial/RTU or libmodbus-backed sibling needs nothing from the MB:: codec to implement it.
//
// This was ModbusConnection in rover_hardware_interface.
//
// Contract of transact():
//   - Failures throw. ModbusDiscreteIoClient drops the link on any exception that escapes it.
//   - A reply that arrived is returned as-is - possibly empty, possibly non-coil cells - and
//     judged by the client, which rejects it without dropping the link.
//   - For WRITE_SINGLE_COIL an empty reply is returned; the device's echo is not decoded.
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

    virtual DiscreteReply transact(const DiscreteRequest & request) = 0;

    virtual void close() = 0;
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_DOMAIN_MODBUS_TRANSPORT_PORT_HPP_
