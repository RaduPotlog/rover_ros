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
// Modified 2026 by Mechatronics Academy: split out of serial_driver/serial_port.hpp and
// serial_driver/src/serial_bridge_node.cpp (ros-drivers/transport_drivers v1.2.0).

#include "rover_serial_driver/domain/serial_port_config.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace rover::transport::serial
{

std::optional<FlowControl> flowControlFromString(std::string_view value)
{
    if (value == "none") {
        return FlowControl::NONE;
    }
    if (value == "hardware") {
        return FlowControl::HARDWARE;
    }
    if (value == "software") {
        return FlowControl::SOFTWARE;
    }
    return std::nullopt;
}

std::optional<Parity> parityFromString(std::string_view value)
{
    if (value == "none") {
        return Parity::NONE;
    }
    if (value == "odd") {
        return Parity::ODD;
    }
    if (value == "even") {
        return Parity::EVEN;
    }
    return std::nullopt;
}

std::optional<StopBits> stopBitsFromString(std::string_view value)
{
    if (value == "1" || value == "1.0") {
        return StopBits::ONE;
    }
    if (value == "1.5") {
        return StopBits::ONE_POINT_FIVE;
    }
    if (value == "2" || value == "2.0") {
        return StopBits::TWO;
    }
    return std::nullopt;
}

SerialPortConfig::SerialPortConfig(
    std::uint32_t baud_rate,
    FlowControl flow_control,
    Parity parity,
    StopBits stop_bits)
: baud_rate_{baud_rate},
  flow_control_{flow_control},
  parity_{parity},
  stop_bits_{stop_bits}
{
}

std::optional<SerialPortConfig> SerialPortConfig::fromStrings(
    int baud_rate,
    std::string_view flow_control,
    std::string_view parity,
    std::string_view stop_bits,
    std::string & error)
{
    // Upstream never checked this, so a missing baud_rate parameter defaulted to 0 and the
    // failure surfaced later as an opaque ASIO error when the port was opened.
    if (baud_rate <= 0) {
        error = "baud_rate must be a positive integer, got " + std::to_string(baud_rate);
        return std::nullopt;
    }

    const auto parsed_flow_control = flowControlFromString(flow_control);
    if (!parsed_flow_control) {
        error = "flow_control must be one of: none, software, hardware; got '" +
            std::string{flow_control} + "'";
        return std::nullopt;
    }

    const auto parsed_parity = parityFromString(parity);
    if (!parsed_parity) {
        error = "parity must be one of: none, odd, even; got '" + std::string{parity} + "'";
        return std::nullopt;
    }

    const auto parsed_stop_bits = stopBitsFromString(stop_bits);
    if (!parsed_stop_bits) {
        error = "stop_bits must be one of: 1, 1.5, 2; got '" + std::string{stop_bits} + "'";
        return std::nullopt;
    }

    return SerialPortConfig{
        static_cast<std::uint32_t>(baud_rate),
        *parsed_flow_control,
        *parsed_parity,
        *parsed_stop_bits};
}

std::uint32_t SerialPortConfig::getBaudRate() const
{
    return baud_rate_;
}

FlowControl SerialPortConfig::getFlowControl() const
{
    return flow_control_;
}

Parity SerialPortConfig::getParity() const
{
    return parity_;
}

StopBits SerialPortConfig::getStopBits() const
{
    return stop_bits_;
}

}  // namespace rover::transport::serial
