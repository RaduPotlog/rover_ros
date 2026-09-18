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

#ifndef ROVER_MODBUS_DRIVER_APPLICATION_MODBUS_DISCRETE_IO_CLIENT_HPP_
#define ROVER_MODBUS_DRIVER_APPLICATION_MODBUS_DISCRETE_IO_CLIENT_HPP_

#include <cstdint>
#include <functional>
#include <memory>

#include <MB/modbusRequest.hpp>
#include <MB/modbusResponse.hpp>

#include "rover_modbus_driver/domain/client_settings.hpp"
#include "rover_modbus_driver/domain/contact_coil_types.hpp"
#include "rover_modbus_driver/domain/discrete_io_port.hpp"
#include "rover_modbus_driver/domain/logger_port.hpp"
#include "rover_modbus_driver/domain/modbus_transport_port.hpp"

namespace rover::transport::modbus
{

// Reads and writes single discrete contacts and coils over an injected transport,
// retrying the initial connection per ClientSettings.
//
// This was RoverModbus in rover_hardware_interface. Two things changed in the move:
// the rclcpp::Logger member became an injected LoggerPort, and the transport is now
// supplied by a factory rather than constructed in place. The latter is what makes the
// wire encoding testable at all - see test/unit/test_modbus_discrete_io_client.cpp,
// which asserts function codes and addresses against a fake transport. Use
// makeModbusTcpDiscreteIoClient() for the ordinary TCP case.
class ModbusDiscreteIoClient : public DiscreteIoPort
{

public:

    // Modified 2026 by Mechatronics Academy: was a public non-static `const uint8_t`
    // data member, which cost per-instance storage and suppressed the implicit
    // copy-assignment operator.
    static constexpr uint8_t kModbusDeviceId = 255U;

    using TransportFactory = std::function<std::unique_ptr<ModbusTransportPort>()>;

    // Throws std::invalid_argument on an empty host, std::runtime_error if the
    // connection cannot be established within the configured number of attempts.
    ModbusDiscreteIoClient(
        TransportFactory transport_factory,
        const ClientSettings & settings,
        std::shared_ptr<LoggerPort> logger);

    ~ModbusDiscreteIoClient() override;

    uint16_t readDiscreteContact(const ContactInfo & contact) override;

    uint16_t readDiscreteCoil(const CoilInfo & coil) override;

    void writeDiscreteCoil(const CoilInfo & coil, const bool coil_state) override;

private:

    MB::ModbusResponse sendRequest(const MB::ModbusRequest & request);

    // Reads the single coil value out of a response, or kDiscreteReadUnavailable if the
    // device answered with something else.
    uint16_t firstCoilValue(const MB::ModbusResponse & response) const;

    std::unique_ptr<ModbusTransportPort> transport_;

    std::shared_ptr<LoggerPort> logger_;
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_APPLICATION_MODBUS_DISCRETE_IO_CLIENT_HPP_
