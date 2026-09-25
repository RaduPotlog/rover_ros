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

#ifndef ROVER_MODBUS_DRIVER_TEST_FAKES_FAKE_MODBUS_TRANSPORT_HPP_
#define ROVER_MODBUS_DRIVER_TEST_FAKES_FAKE_MODBUS_TRANSPORT_HPP_

#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <MB/modbusCell.hpp>
#include <MB/modbusException.hpp>
#include <MB/modbusRequest.hpp>
#include <MB/modbusResponse.hpp>
#include <MB/modbusUtils.hpp>

#include "rover_modbus_driver/domain/discrete_transaction.hpp"
#include "rover_modbus_driver/domain/modbus_transport_port.hpp"
#include "rover_modbus_driver/infrastructure/mb_frame_mapping.hpp"

namespace rover::transport::modbus::test
{

// Records every request the client hands down and replies with a canned response, so the
// wire encoding can be asserted without a socket. The recording lives in a shared
// Journal rather than in the transport itself because the client owns its transport and
// the test cannot reach inside it afterwards.
//
// It runs the production mapping (infrastructure/mb_frame_mapping) in both directions, exactly
// as ModbusTcpTransport does, so every assertion on journal->requests still checks the
// MB::ModbusRequest the TCP transport would put on the wire.
struct Journal
{
    std::vector<MB::ModbusRequest> requests;

    // Value the next read should report back.
    bool coil_value{true};

    // When set, a read replies with exactly these cells instead of the single coil_value - for
    // the batched reads, including replies padded past or cut short of the requested count.
    std::optional<std::vector<bool>> bit_values;

    // When set, transact() throws this instead of answering.
    std::optional<MB::utils::MBErrorCode> throw_error;

    // Return a response with no register values, to exercise the short-reply guard.
    bool reply_empty{false};

    bool closed{false};

    // How many transports the factory has handed out. The client re-dials by calling its factory
    // again, so this counts reconnections (the first one is the constructor's).
    int transports_created{0};

    // When set, the factory throws instead of producing a transport - a host that is refusing
    // connections.
    bool factory_fails{false};
};

class FakeModbusTransport : public ModbusTransportPort
{

public:

    explicit FakeModbusTransport(std::shared_ptr<Journal> journal)
    : journal_(std::move(journal))
    {
    }

    DiscreteReply transact(const DiscreteRequest & request) override
    {
        const MB::ModbusRequest req = toMbRequest(request);
        journal_->requests.push_back(req);

        if (journal_->throw_error.has_value()) {
            throw MB::ModbusException(*journal_->throw_error);
        }

        // Like ModbusTcpTransport: a write's echo is not decoded.
        if (request.function == DiscreteFunction::WRITE_SINGLE_COIL) {
            return DiscreteReply{};
        }

        std::vector<MB::ModbusCell> values;

        if (journal_->bit_values.has_value()) {
            for (const bool bit : *journal_->bit_values) {
                values.push_back(MB::ModbusCell(bit));
            }
        } else if (!journal_->reply_empty) {
            values.push_back(MB::ModbusCell(journal_->coil_value));
        }

        // A prvalue bound straight to toDiscreteReply()'s const reference - never a copied
        // MB::ModbusResponse, whose copy constructor throws on an empty reply.
        return toDiscreteReply(MB::ModbusResponse(
            req.slaveID(), req.functionCode(), req.registerAddress(), req.numberOfRegisters(),
            values));
    }

    void close() override { journal_->closed = true; }

private:

    std::shared_ptr<Journal> journal_;
};

}  // namespace rover::transport::modbus::test

#endif  // ROVER_MODBUS_DRIVER_TEST_FAKES_FAKE_MODBUS_TRANSPORT_HPP_
