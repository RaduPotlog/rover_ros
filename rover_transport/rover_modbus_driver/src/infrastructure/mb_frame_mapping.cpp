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

#include "rover_modbus_driver/infrastructure/mb_frame_mapping.hpp"

#include <vector>

#include <MB/modbusCell.hpp>
#include <MB/modbusException.hpp>
#include <MB/modbusUtils.hpp>

namespace rover::transport::modbus
{

// DiscreteFunction's enumerators are the wire function codes, so toMbRequest() - and the
// client's exception function codes - can cast rather than switch.
static_assert(
    static_cast<uint8_t>(DiscreteFunction::READ_COILS) == MB::utils::ReadDiscreteOutputCoils,
    "READ_COILS must be FC1");
static_assert(
    static_cast<uint8_t>(DiscreteFunction::READ_DISCRETE_INPUTS) ==
    MB::utils::ReadDiscreteInputContacts,
    "READ_DISCRETE_INPUTS must be FC2");
static_assert(
    static_cast<uint8_t>(DiscreteFunction::WRITE_SINGLE_COIL) ==
    MB::utils::WriteSingleDiscreteOutputCoil,
    "WRITE_SINGLE_COIL must be FC5");

MB::ModbusRequest toMbRequest(const DiscreteRequest & request)
{
    const auto function_code = static_cast<MB::utils::MBFunctionCode>(request.function);

    if (request.function == DiscreteFunction::WRITE_SINGLE_COIL) {
        return MB::ModbusRequest(
            request.unit_id, function_code, request.address, request.count,
            std::vector<MB::ModbusCell>{MB::ModbusCell(request.coil_value)});
    }

    return MB::ModbusRequest(request.unit_id, function_code, request.address, request.count);
}

DiscreteReply toDiscreteReply(const MB::ModbusResponse & response)
{
    // Never copy an MB::ModbusResponse lvalue here. Its copy constructor calls registerValues(),
    // which throws on an empty reply - so `response` is taken by const reference and never
    // re-bound to a local MB::ModbusResponse by value.
    const std::vector<MB::ModbusCell> * values = nullptr;

    try {
        values = &response.registerValues();
    } catch (const MB::ModbusException &) {
        // registerValues() throws NumberOfValuesInvalid exactly when the reply has no values.
        // That is an empty DiscreteReply: the client, not the transport, rejects it.
        return DiscreteReply{};
    }

    DiscreteReply reply{};
    reply.cells.reserve(values->size());

    for (const MB::ModbusCell & cell : *values) {
        reply.cells.push_back(
            cell.isCoil() ? ReplyCell{true, cell.coil()} : ReplyCell{false, false});
    }

    return reply;
}

}  // namespace rover::transport::modbus
