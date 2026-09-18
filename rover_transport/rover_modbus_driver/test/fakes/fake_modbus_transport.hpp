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
#include <MB/modbusUtils.hpp>

#include "rover_modbus_driver/domain/modbus_transport_port.hpp"

namespace rover::transport::modbus::test
{

// Records every request the client hands down and replies with a canned response, so the
// wire encoding can be asserted without a socket. The recording lives in a shared
// Journal rather than in the transport itself because the client owns its transport and
// the test cannot reach inside it afterwards.
struct Journal
{
    std::vector<MB::ModbusRequest> requests;

    // Value the next read should report back.
    bool coil_value{true};

    // When set, sendRequest throws this instead of answering.
    std::optional<MB::utils::MBErrorCode> throw_error;

    // Return a response with no register values, to exercise the short-reply guard.
    bool reply_empty{false};

    bool closed{false};
};

class FakeModbusTransport : public ModbusTransportPort
{

public:

    explicit FakeModbusTransport(std::shared_ptr<Journal> journal)
    : journal_(std::move(journal))
    {
    }

    MB::ModbusResponse sendRequest(const MB::ModbusRequest & req) override
    {
        journal_->requests.push_back(req);

        if (journal_->throw_error.has_value()) {
            throw MB::ModbusException(*journal_->throw_error);
        }

        std::vector<MB::ModbusCell> values;

        if (!journal_->reply_empty) {
            values.push_back(MB::ModbusCell(journal_->coil_value));
        }

        return MB::ModbusResponse(
            req.slaveID(), req.functionCode(), req.registerAddress(), req.numberOfRegisters(),
            values);
    }

    void close() override { journal_->closed = true; }

private:

    std::shared_ptr<Journal> journal_;
};

}  // namespace rover::transport::modbus::test

#endif  // ROVER_MODBUS_DRIVER_TEST_FAKES_FAKE_MODBUS_TRANSPORT_HPP_
