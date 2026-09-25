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

#ifndef ROVER_MODBUS_DRIVER_INFRASTRUCTURE_MODBUS_TCP_TRANSPORT_HPP_
#define ROVER_MODBUS_DRIVER_INFRASTRUCTURE_MODBUS_TCP_TRANSPORT_HPP_

#include <string>

#include <MB/connection.hpp>
#include <MB/modbusRequest.hpp>
#include <MB/modbusResponse.hpp>

#include "rover_modbus_driver/domain/discrete_transaction.hpp"
#include "rover_modbus_driver/domain/modbus_transport_port.hpp"

namespace rover::transport::modbus
{

// ModbusTransportPort over a real TCP socket. This was ModbusTcpConnection in
// rover_hardware_interface.
//
// What stays here: the socket, its response timeout, close(), and the MB:: transaction itself.
// transact() translates through infrastructure/mb_frame_mapping on either side of it.
class ModbusTcpTransport : public ModbusTransportPort
{

public:

    // Throws std::runtime_error if the connection cannot be opened.
    ModbusTcpTransport(const std::string & host, int port, unsigned response_timeout_ms);

    ~ModbusTcpTransport() override;

    DiscreteReply transact(const DiscreteRequest & request) override;

    // The raw MB:: transaction transact() is built on. Public for
    // test/e2e/test_modbus_tcp_roundtrip.cpp; no longer an override.
    MB::ModbusResponse sendRequest(const MB::ModbusRequest & req);

    void close() override;

private:

    MB::TCP::Connection connection_;

    bool open_{true};
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_INFRASTRUCTURE_MODBUS_TCP_TRANSPORT_HPP_
