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
#include <algorithm>
#include <string>
#include <thread>
#include <vector>

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

// The function code an MB::ModbusException for this transaction carries. DiscreteFunction's
// enumerators are defined as the wire function codes (checked by the static_asserts in
// infrastructure/mb_frame_mapping.cpp), so this is a cast.
MB::utils::MBFunctionCode mbFunctionCode(const DiscreteFunction function)
{
    return static_cast<MB::utils::MBFunctionCode>(function);
}

}  // namespace

ModbusDiscreteIoClient::ModbusDiscreteIoClient(
    TransportFactory transport_factory,
    const ClientSettings & settings,
    std::shared_ptr<LoggerPort> logger)
: transport_factory_(transport_factory), settings_(settings), logger_(std::move(logger))
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

bool ModbusDiscreteIoClient::isConnected() const
{
    return transport_ != nullptr;
}

void ModbusDiscreteIoClient::dropTransport()
{
    if (!transport_) {
        return;
    }

    try {
        transport_->close();
    } catch (...) {
        // Closing a socket that is already broken is not worth reporting, and this runs from an
        // error path that is about to rethrow something more useful.
    }

    transport_.reset();

    // First failure gets the configured delay; each subsequent one doubles, capped. Starting at
    // the configured delay rather than zero keeps a flapping link from being re-dialled on every
    // single transaction.
    reconnect_backoff_ = (reconnect_backoff_.count() == 0)
        ? std::chrono::milliseconds(std::max(settings_.connection_retry_delay_ms, 1u))
        : std::min(
              reconnect_backoff_ * 2,
              std::chrono::duration_cast<std::chrono::milliseconds>(kMaxReconnectBackoff));

    next_reconnect_at_ = std::chrono::steady_clock::now() + reconnect_backoff_;

    logger_->warn(
        "Modbus link to " + settings_.host + ":" + std::to_string(settings_.port) +
        " dropped; next reconnection attempt in " + std::to_string(reconnect_backoff_.count()) +
        " ms.");
}

void ModbusDiscreteIoClient::ensureConnected()
{
    if (transport_) {
        return;
    }

    if (std::chrono::steady_clock::now() < next_reconnect_at_) {
        // Inside the backoff window: fail fast rather than dial. See the header for why an
        // attempt is expensive enough to be worth rationing.
        throw std::runtime_error(
            "Modbus link to " + settings_.host + ":" + std::to_string(settings_.port) +
            " is down; waiting out the reconnection backoff.");
    }

    try {
        transport_ = transport_factory_();
    } catch (const std::exception & e) {
        next_reconnect_at_ = std::chrono::steady_clock::now() + reconnect_backoff_;
        throw std::runtime_error(
            "Modbus reconnection to " + settings_.host + ":" + std::to_string(settings_.port) +
            " failed: " + e.what());
    }

    if (!transport_) {
        next_reconnect_at_ = std::chrono::steady_clock::now() + reconnect_backoff_;
        throw std::runtime_error(
            "Modbus reconnection to " + settings_.host + ":" + std::to_string(settings_.port) +
            " produced no transport.");
    }

    reconnect_backoff_ = std::chrono::milliseconds(0);
    logger_->warn(
        "Modbus link to " + settings_.host + ":" + std::to_string(settings_.port) + " restored.");
}

ModbusDiscreteIoClient::~ModbusDiscreteIoClient()
{
    if (transport_) {
        transport_->close();
    }
}

uint16_t ModbusDiscreteIoClient::firstCoilValue(const DiscreteReply & reply) const
{
    // A reply with no values is rejected here, with the same exception
    // MB::ModbusResponse::registerValues() used to throw for it: NumberOfValuesInvalid, slave
    // 0xFF, no function code. That throw used to come out of registerValues() before this
    // function could look at the vector; now toDiscreteReply() turns the same empty reply into
    // an empty DiscreteReply one layer down, so the check and the throw happen here instead.
    // Either way it is thrown after the transaction, so it is caught, logged and rethrown by the
    // callers without dropping the link.
    //
    // The non-coil guard is a defensive backstop that is not reachable through the current
    // codec: for the discrete function codes this client uses, ModbusResponse's constructor
    // coerces every cell via ModbusCell::coil(), and fromRaw() builds coil cells directly. That
    // leaves kDiscreteReadUnavailable effectively dead - as the 255U sentinel it replaced always
    // was. Written down so nobody "fixes" it into a silent failure later.
    if (reply.cells.empty()) {
        throw MB::ModbusException(MB::utils::NumberOfValuesInvalid);
    }

    if (!reply.cells.front().is_coil) {
        return kDiscreteReadUnavailable;
    }

    // A coil read returns a whole byte's worth of bits, so the reply carries 8 cells per byte
    // regardless of how many were requested (see ModbusResponse::fromRaw). We ask for one coil
    // starting at the target address, so bit 0 - front() - is it.
    return reply.cells.front().value;
}

uint16_t ModbusDiscreteIoClient::readDiscreteContact(const ContactInfo & contact)
{
    const DiscreteRequest request{
        kModbusDeviceId, DiscreteFunction::READ_DISCRETE_INPUTS,
        static_cast<uint16_t>(contact.contact), 1, false};

    try {
        return firstCoilValue(sendRequest(request));
    } catch (const MB::ModbusException &) {
        logger_->error("Failed to read contact");
        throw;
    }
}

uint16_t ModbusDiscreteIoClient::readDiscreteCoil(const CoilInfo & coil)
{
    const DiscreteRequest request{
        kModbusDeviceId, DiscreteFunction::READ_COILS, static_cast<uint16_t>(coil.coil), 1,
        false};

    try {
        return firstCoilValue(sendRequest(request));
    } catch (const MB::ModbusException &) {
        logger_->error("Failed to read coil");
        throw;
    }
}

std::vector<bool> ModbusDiscreteIoClient::readBits(
    const DiscreteFunction function, const uint16_t first_address, const uint16_t count)
{
    const DiscreteRequest request{kModbusDeviceId, function, first_address, count, false};

    const DiscreteReply reply = sendRequest(request);
    const auto & cells = reply.cells;

    // No values at all: the exception registerValues() used to throw, without a function code.
    if (cells.empty()) {
        throw MB::ModbusException(MB::utils::NumberOfValuesInvalid);
    }

    if (cells.size() < count) {
        throw MB::ModbusException(
            MB::utils::NumberOfValuesInvalid, kModbusDeviceId, mbFunctionCode(function));
    }

    std::vector<bool> bits(count);

    for (uint16_t i = 0; i < count; ++i) {
        if (!cells[i].is_coil) {
            throw MB::ModbusException(
                MB::utils::NumberOfValuesInvalid, kModbusDeviceId, mbFunctionCode(function));
        }

        bits[i] = cells[i].value;
    }

    return bits;
}

std::vector<bool> ModbusDiscreteIoClient::readDiscreteContacts(
    const Contact first, const uint16_t count)
{
    try {
        return readBits(
            DiscreteFunction::READ_DISCRETE_INPUTS, static_cast<uint16_t>(first), count);
    } catch (const MB::ModbusException &) {
        logger_->error("Failed to read contacts");
        throw;
    }
}

std::vector<bool> ModbusDiscreteIoClient::readDiscreteCoils(const Coil first, const uint16_t count)
{
    try {
        return readBits(DiscreteFunction::READ_COILS, static_cast<uint16_t>(first), count);
    } catch (const MB::ModbusException &) {
        logger_->error("Failed to read coils");
        throw;
    }
}

void ModbusDiscreteIoClient::writeDiscreteCoil(const CoilInfo & coil, const bool coil_state)
{
    if (!coil.is_coil_engage_allowed) {
        logger_->error("Coil engage is not allowed");
        return;
    }

    const DiscreteRequest request{
        kModbusDeviceId, DiscreteFunction::WRITE_SINGLE_COIL, static_cast<uint16_t>(coil.coil),
        1, coil_state};

    try {
        (void)sendRequest(request);
    } catch (const MB::ModbusException &) {
        logger_->error("Failed to write coil");
        throw;
    }
}

DiscreteReply ModbusDiscreteIoClient::sendRequest(const DiscreteRequest & request)
{
    ensureConnected();

    try {
        return transport_->transact(request);
    } catch (const MB::ModbusException & ex) {
        logger_->error(std::string("Modbus exception: ") + ex.what());
        dropTransport();
        throw;
    } catch (...) {
        dropTransport();
        throw;
    }
}

}  // namespace rover::transport::modbus
