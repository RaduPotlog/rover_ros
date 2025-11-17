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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_MODBUS_MODBUS_TCP_CONNECTION_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_MODBUS_MODBUS_TCP_CONNECTION_HPP_

#include <string>

#include <connection.hpp>

#include "rover_hardware_interface/rover_modbus/modbus_connection.hpp"

namespace rover_hardware_interface
{

class ModbusTcpConnection : public ModbusConnection 
{

public:
    
    ModbusTcpConnection(const std::string &ip, int port) 
    : connection(MB::TCP::Connection::with(ip, port))
    {

    }

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override 
    {
        try {
            connection.sendRequest(req);
            return connection.awaitResponse();
        } catch (const MB::ModbusException &ex) {
            throw;
        }
    }

    void close() override 
    {
        // Do nothing at the moment. Can explicitly close the connection if needed.
    }

private:
    
    MB::TCP::Connection connection;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_MODBUS_MODBUS_TCP_CONNECTION_HPP_