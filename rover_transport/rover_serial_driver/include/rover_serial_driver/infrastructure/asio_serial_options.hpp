// Copyright 2021 LeoDrive, Copyright 2021 The Autoware Foundation
// Copyright 2021 Trimble (c)
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
//
// Modified 2026 by Mechatronics Academy: extracted from the get_*_asio() members of
// SerialPortConfig (ros-drivers/transport_drivers v1.2.0) into free functions, so the
// domain value object no longer depends on ASIO.

#ifndef ROVER_SERIAL_DRIVER_INFRASTRUCTURE_ASIO_SERIAL_OPTIONS_HPP_
#define ROVER_SERIAL_DRIVER_INFRASTRUCTURE_ASIO_SERIAL_OPTIONS_HPP_

#include <asio.hpp>

#include "rover_serial_driver/domain/serial_port_config.hpp"

namespace rover::transport::serial
{

using SerialPortBase = asio::serial_port_base;

SerialPortBase::baud_rate toAsioBaudRate(const SerialPortConfig & config);

SerialPortBase::flow_control::type toAsioFlowControl(FlowControl flow_control);

SerialPortBase::parity::type toAsioParity(Parity parity);

SerialPortBase::stop_bits::type toAsioStopBits(StopBits stop_bits);

}  // namespace rover::transport::serial

#endif  // ROVER_SERIAL_DRIVER_INFRASTRUCTURE_ASIO_SERIAL_OPTIONS_HPP_
