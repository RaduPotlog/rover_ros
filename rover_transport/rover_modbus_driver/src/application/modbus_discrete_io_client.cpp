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

#include "rover_modbus_driver/application/modbus_discrete_io_client.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <MB/modbusCell.hpp>
#include <MB/modbusException.hpp>
#include <MB/modbusUtils.hpp>

namespace rover::transport::modbus
{

namespace
{

// A local retry loop rather than rover_hardware_interface's operationWithAttempts():
// depending on that package from here would invert the dependency this extraction exists
// to establish. The semantics are the same - attempts, a delay between them, and a bool
// for whether any attempt succeeded.
bool connectWithAttempts(
    const std::function<void()> & attempt,
    const unsigned max_attempts,
    const std::chrono::milliseconds delay_between_attempts,
    LoggerPort & logger)
{
    for (unsigned n = 0; n < max_attempts; ++n) {
        try {
            attempt();
            return true;
        } catch (const std::exception & e) {
            logger.warn(
                "Modbus connection attempt " + std::to_string(n + 1) + " failed: " + e.what());
        } catch (...) {
            logger.warn("Modbus connection attempt " + std::to_string(n + 1) + " failed.");
        }

        if (delay_between_attempts.count() > 0) {
            std::this_thread::sleep_for(delay_between_attempts);
        }
    }

    return false;
}

}  // namespace

ModbusDiscreteIoClient::ModbusDiscreteIoClient(
    TransportFactory transport_factory,
    const ClientSettings & settings,
    std::shared_ptr<LoggerPort> logger)
: logger_(std::move(logger))
{
    if (!transport_factory) {
        throw std::invalid_argument("A transport factory is required.");
    }

    if (!logger_) {
        throw std::invalid_argument("A logger is required.");
    }

    if (settings.host.empty()) {
        throw std::invalid_argument("Please provide an IP address for TCP connection.");
    }

    // A configured attempt count of 0 means "retry forever", matching the previous
    // unconditional while(true). Translated here rather than giving 0 a second meaning
    // in the retry helper, where "0 attempts" naturally reads as "don't even try".
    const unsigned attempts = (settings.connection_retry_count == 0)
        ? std::numeric_limits<unsigned>::max()
        : settings.connection_retry_count;

    const bool connected = connectWithAttempts(
        [this, &transport_factory]() { transport_ = transport_factory(); },
        attempts,
        std::chrono::milliseconds(settings.connection_retry_delay_ms),
        *logger_);

    if (!connected || !transport_) {
        throw std::runtime_error(
            "Failed to establish Modbus TCP connection to " + settings.host + ":" +
            std::to_string(settings.port) + " after " +
            std::to_string(settings.connection_retry_count) + " attempts.");
    }
}

ModbusDiscreteIoClient::~ModbusDiscreteIoClient()
{
    if (transport_) {
        transport_->close();
    }
}

uint16_t ModbusDiscreteIoClient::firstCoilValue(const MB::ModbusResponse & response) const
{
    // Note on the two guards below: neither is reachable through the current codec, and
    // that is worth writing down so nobody "fixes" it into a silent failure later.
    //
    //   - MB::ModbusResponse::registerValues() throws NumberOfValuesInvalid when the
    //     vector is empty, so a short reply surfaces as an MB::ModbusException from the
    //     line below rather than reaching the empty() test. That is the behaviour we
    //     want - it is caught, logged and rethrown by the callers.
    //   - For the discrete function codes this client uses, values are always coils:
    //     ModbusResponse's constructor coerces every cell via ModbusCell::coil(), and
    //     fromRaw() builds coil cells directly. So isCoil() cannot be false here.
    //
    // They stay as a defensive backstop, which also means kDiscreteReadUnavailable is
    // effectively dead - as the 255U sentinel it replaced always was.
    const auto & values = response.registerValues();

    if (values.empty() || !values.front().isCoil()) {
        return kDiscreteReadUnavailable;
    }

    // A coil read returns a whole byte's worth of bits, so the response carries 8 cells
    // per byte regardless of how many were requested (see ModbusResponse::fromRaw). We
    // ask for one coil starting at the target address, so bit 0 - front() - is it.
    return values.front().coil();
}

uint16_t ModbusDiscreteIoClient::readDiscreteContact(const ContactInfo & contact)
{
    MB::ModbusRequest request(
        kModbusDeviceId, MB::utils::ReadDiscreteInputContacts,
        static_cast<uint16_t>(contact.contact), 1);

    try {
        return firstCoilValue(sendRequest(request));
    } catch (const MB::ModbusException &) {
        logger_->error("Failed to read contact");
        throw;
    }
}

uint16_t ModbusDiscreteIoClient::readDiscreteCoil(const CoilInfo & coil)
{
    MB::ModbusRequest request(
        kModbusDeviceId, MB::utils::ReadDiscreteOutputCoils,
        static_cast<uint16_t>(coil.coil), 1);

    try {
        return firstCoilValue(sendRequest(request));
    } catch (const MB::ModbusException &) {
        logger_->error("Failed to read coil");
        throw;
    }
}

void ModbusDiscreteIoClient::writeDiscreteCoil(const CoilInfo & coil, const bool coil_state)
{
    if (!coil.is_coil_engage_allowed) {
        logger_->error("Coil engage is not allowed");
        return;
    }

    const std::vector<MB::ModbusCell> value = {MB::ModbusCell(coil_state)};

    MB::ModbusRequest request(
        kModbusDeviceId, MB::utils::WriteSingleDiscreteOutputCoil,
        static_cast<uint16_t>(coil.coil), 1, value);

    try {
        (void)sendRequest(request);
    } catch (const MB::ModbusException &) {
        logger_->error("Failed to write coil");
        throw;
    }
}

MB::ModbusResponse ModbusDiscreteIoClient::sendRequest(const MB::ModbusRequest & request)
{
    try {
        return transport_->sendRequest(request);
    } catch (const MB::ModbusException & ex) {
        logger_->error(std::string("Modbus exception: ") + ex.what());
        throw;
    }
}

}  // namespace rover::transport::modbus
