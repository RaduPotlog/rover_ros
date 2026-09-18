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

#include "rover_modbus_driver/infrastructure/modbus_tcp_transport.hpp"

namespace rover::transport::modbus
{

ModbusTcpTransport::ModbusTcpTransport(
    const std::string & host, const int port, const unsigned response_timeout_ms)
: connection_(MB::TCP::Connection::with(host, port))
{
    connection_.setTimeout(static_cast<int>(response_timeout_ms));
}

ModbusTcpTransport::~ModbusTcpTransport()
{
    close();
}

MB::ModbusResponse ModbusTcpTransport::sendRequest(const MB::ModbusRequest & req)
{
    // Modified 2026 by Mechatronics Academy: dropped a try/catch that caught
    // MB::ModbusException into an unused variable and immediately rethrew it.
    connection_.sendRequest(req);

    return connection_.awaitResponse();
}

void ModbusTcpTransport::close()
{
    // Modified 2026 by Mechatronics Academy: this used to be an explicit no-op with a
    // comment saying the connection could be closed "if needed". Combined with the
    // missing virtual destructor on the port, that meant nothing ever closed the socket.
    // MB::TCP::Connection is RAII and closes in its own destructor; assigning a default
    // -constructed Connection makes an explicit close() actually release the fd, and
    // makes a second call harmless.
    if (!open_) {
        return;
    }

    connection_ = MB::TCP::Connection();
    open_       = false;
}

}  // namespace rover::transport::modbus
