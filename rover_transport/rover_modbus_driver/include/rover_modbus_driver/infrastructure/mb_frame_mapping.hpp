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

#ifndef ROVER_MODBUS_DRIVER_INFRASTRUCTURE_MB_FRAME_MAPPING_HPP_
#define ROVER_MODBUS_DRIVER_INFRASTRUCTURE_MB_FRAME_MAPPING_HPP_

#include <MB/modbusRequest.hpp>
#include <MB/modbusResponse.hpp>

#include "rover_modbus_driver/domain/discrete_transaction.hpp"

namespace rover::transport::modbus
{

// The only translation between this package's DiscreteRequest/DiscreteReply and the MB:: codec's
// frame types. Socket-free and ROS-free, in its own library (rover_modbus_driver_mb_mapping), so
// ModbusTcpTransport and the unit tests' fake transport both run exactly this code.

// Builds the MB::ModbusRequest the client used to construct directly: the same constructor
// arguments, so the same bytes on the wire.
MB::ModbusRequest toMbRequest(const DiscreteRequest & request);

// Every cell of `response`, byte padding included; register cells become is_coil = false. A
// response with no values becomes an empty DiscreteReply rather than throwing, so the client can
// reject it without dropping the link.
DiscreteReply toDiscreteReply(const MB::ModbusResponse & response);

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_INFRASTRUCTURE_MB_FRAME_MAPPING_HPP_
