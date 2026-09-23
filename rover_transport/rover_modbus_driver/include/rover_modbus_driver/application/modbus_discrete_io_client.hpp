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

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <MB/modbusRequest.hpp>
#include <MB/modbusResponse.hpp>
#include <MB/modbusUtils.hpp>

#include "rover_modbus_driver/domain/client_settings.hpp"
#include "rover_modbus_driver/domain/contact_coil_types.hpp"
#include "rover_modbus_driver/domain/discrete_io_port.hpp"
#include "rover_modbus_driver/domain/logger_port.hpp"
#include "rover_modbus_driver/domain/modbus_transport_port.hpp"

namespace rover::transport::modbus
{

// Reads and writes single discrete contacts and coils over an injected transport,
// retrying the initial connection per ClientSettings and re-dialling it if it later drops.
//
// Reconnection matters because this client sits on the rover's safety path. Previously the
// transport was built once in the constructor and never rebuilt: the retry-forever setting
// applied only to that first dial, so any later link drop was permanent until the whole
// ros2_control component was re-configured - and since every failure rethrows, the first
// post-configure hiccup also took down the process via an uncaught exception in a background
// thread.
//
// The reconnect is deliberately lazy and backed off rather than eager. MB::TCP::Connection::with()
// performs a blocking ::connect() with no timeout, so an attempt against a black-holed host can
// occupy the caller for the kernel's SYN-retry period. Between attempts, operations therefore
// fail fast instead of dialling; at most one attempt is made per backoff window, and the window
// grows to kMaxReconnectBackoff. When the link really is down the safety relay has already
// latched the E-Stop (its watchdog heartbeat stopped arriving), so a slow attempt costs nothing
// that is not already lost.
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

    // Longest gap between reconnection attempts. Reached by doubling from
    // ClientSettings::connection_retry_delay_ms.
    static constexpr std::chrono::seconds kMaxReconnectBackoff{30};

    // Whether the transport is currently established. False between a failed operation and a
    // successful re-dial.
    bool isConnected() const;

    uint16_t readDiscreteContact(const ContactInfo & contact) override;

    uint16_t readDiscreteCoil(const CoilInfo & coil) override;

    void writeDiscreteCoil(const CoilInfo & coil, const bool coil_state) override;

    std::vector<bool> readDiscreteContacts(const Contact first, const uint16_t count) override;

    std::vector<bool> readDiscreteCoils(const Coil first, const uint16_t count) override;

private:

    MB::ModbusResponse sendRequest(const MB::ModbusRequest & request);

    // Drops the transport so the next operation re-dials. Called when an operation fails: a
    // Modbus exception can mean a timeout on a live socket or a dead one, and re-dialling on
    // either is cheap relative to leaving a half-dead link in place.
    void dropTransport();

    // Re-dials if the backoff window has elapsed. Throws if still disconnected afterwards, so
    // callers keep seeing an exception per failed operation exactly as before.
    void ensureConnected();

    // Reads the single coil value out of a response, or kDiscreteReadUnavailable if the
    // device answered with something else.
    uint16_t firstCoilValue(const MB::ModbusResponse & response) const;

    // One FC1/FC2 transaction for `count` consecutive bits. The reply is padded to whole bytes,
    // so it carries more cells than asked for; only the first `count` are returned.
    std::vector<bool> readBits(
        const MB::utils::MBFunctionCode function_code, const uint16_t first_address,
        const uint16_t count);

    std::unique_ptr<ModbusTransportPort> transport_;

    // Kept so the link can be re-dialled; the constructor used to consume and discard both.
    TransportFactory transport_factory_;
    ClientSettings settings_;

    std::chrono::steady_clock::time_point next_reconnect_at_{};
    std::chrono::milliseconds reconnect_backoff_{0};

    std::shared_ptr<LoggerPort> logger_;
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_APPLICATION_MODBUS_DISCRETE_IO_CLIENT_HPP_
