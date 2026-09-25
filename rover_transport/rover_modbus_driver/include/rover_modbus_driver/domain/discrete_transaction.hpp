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

#ifndef ROVER_MODBUS_DRIVER_DOMAIN_DISCRETE_TRANSACTION_HPP_
#define ROVER_MODBUS_DRIVER_DOMAIN_DISCRETE_TRANSACTION_HPP_

#include <cstdint>
#include <vector>

namespace rover::transport::modbus
{

// One discrete-IO transaction in this package's own terms: what ModbusTransportPort::transact()
// takes and returns. No codec type appears here, so a transport needs nothing from MB:: to
// implement the port. The translation to and from MB:: frames lives in
// infrastructure/mb_frame_mapping.
//
// None of these structs has default member initialisers. Always brace-initialise every field.

// The enumerator value IS the Modbus function code on the wire.
enum class DiscreteFunction : uint8_t
{
    READ_COILS = 0x01,
    READ_DISCRETE_INPUTS = 0x02,
    WRITE_SINGLE_COIL = 0x05,
};

// Reads: `count` objects from `address`, coil_value false. WRITE_SINGLE_COIL: count 1,
// coil_value = the state to drive.
struct DiscreteRequest
{
    uint8_t unit_id;
    DiscreteFunction function;
    uint16_t address;
    uint16_t count;
    bool coil_value;
};

// One decoded reply cell. is_coil is false when the device answered with a register; `value` is
// meaningful only when is_coil.
struct ReplyCell
{
    bool is_coil;
    bool value;
};

// Every cell the device sent, byte padding included (8 per reply byte). Empty when the reply
// carried no values.
struct DiscreteReply
{
    std::vector<ReplyCell> cells;
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_DOMAIN_DISCRETE_TRANSACTION_HPP_
