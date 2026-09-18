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

#ifndef ROVER_MODBUS_DRIVER_INFRASTRUCTURE_MODBUS_TCP_CLIENT_FACTORY_HPP_
#define ROVER_MODBUS_DRIVER_INFRASTRUCTURE_MODBUS_TCP_CLIENT_FACTORY_HPP_

#include <memory>

#include "rover_modbus_driver/domain/client_settings.hpp"
#include "rover_modbus_driver/domain/discrete_io_port.hpp"
#include "rover_modbus_driver/domain/logger_port.hpp"

namespace rover::transport::modbus
{

// The ordinary way to get a working client: a ModbusDiscreteIoClient wired to a real TCP
// transport. Matches the house makeX(...) -> unique_ptr<Interface> convention used by
// makeSerialPort() and makeUdpReceiver().
//
// Passing no logger gets an RclcppLogger on the "RoverModbus" logger name, which is what
// the safety controller logged under before this package existed.
//
// Throws std::invalid_argument on an empty host, std::runtime_error if the connection
// cannot be established within settings.connection_retry_count attempts (0 = forever).
std::unique_ptr<DiscreteIoPort> makeModbusTcpDiscreteIoClient(
    const ClientSettings & settings,
    std::shared_ptr<LoggerPort> logger = nullptr);

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_INFRASTRUCTURE_MODBUS_TCP_CLIENT_FACTORY_HPP_
