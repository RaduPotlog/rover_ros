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

#include "rover_modbus_driver/infrastructure/modbus_tcp_client_factory.hpp"

#include <utility>

#include "rover_modbus_driver/application/modbus_discrete_io_client.hpp"
#include "rover_modbus_driver/infrastructure/modbus_tcp_transport.hpp"
#include "rover_modbus_driver/infrastructure/rclcpp_logger.hpp"

namespace rover::transport::modbus
{

std::unique_ptr<DiscreteIoPort> makeModbusTcpDiscreteIoClient(
    const ClientSettings & settings, std::shared_ptr<LoggerPort> logger)
{
    if (!logger) {
        logger = std::make_shared<RclcppLogger>(rclcpp::get_logger("RoverModbus"));
    }

    auto transport_factory = [settings]() -> std::unique_ptr<ModbusTransportPort> {
        return std::make_unique<ModbusTcpTransport>(
            settings.host, settings.port, settings.response_timeout_ms);
    };

    return std::make_unique<ModbusDiscreteIoClient>(
        std::move(transport_factory), settings, std::move(logger));
}

}  // namespace rover::transport::modbus
