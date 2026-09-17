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
// Modified 2026 by Mechatronics Academy: split out of serial_driver/serial_port.hpp
// (ros-drivers/transport_drivers v1.2.0). The four get_*_asio() accessors moved to
// infrastructure/asio_serial_options.hpp so this header carries no ASIO dependency, and
// the parameter string parsing was lifted here out of SerialBridgeNode::get_params().

#ifndef ROVER_SERIAL_DRIVER_DOMAIN_SERIAL_PORT_CONFIG_HPP_
#define ROVER_SERIAL_DRIVER_DOMAIN_SERIAL_PORT_CONFIG_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace rover::transport::serial
{

enum class FlowControl
{
    NONE,
    HARDWARE,
    SOFTWARE
};

enum class Parity
{
    NONE,
    ODD,
    EVEN
};

enum class StopBits
{
    ONE,
    ONE_POINT_FIVE,
    TWO
};

// The accepted spellings are a frozen contract: they appear in rover_crsf_teleop's
// config and in upstream's documentation. Do not add, rename or drop one without
// treating it as an interface change.
std::optional<FlowControl> flowControlFromString(std::string_view value);

std::optional<Parity> parityFromString(std::string_view value);

std::optional<StopBits> stopBitsFromString(std::string_view value);

// A validated serial line configuration. Pure data - the translation into ASIO option
// objects lives in infrastructure.
class SerialPortConfig
{

public:

    SerialPortConfig(
        std::uint32_t baud_rate,
        FlowControl flow_control,
        Parity parity,
        StopBits stop_bits);

    // Parses the four ROS parameters into a config, or returns nullopt and writes why into
    // `error`. Upstream threw std::invalid_argument from inside a catch block that only
    // handled rclcpp::ParameterTypeException, so a bad flow_control string escaped the
    // constructor as an unhandled exception; returning nullopt lets the node fail its
    // lifecycle transition cleanly instead.
    static std::optional<SerialPortConfig> fromStrings(
        int baud_rate,
        std::string_view flow_control,
        std::string_view parity,
        std::string_view stop_bits,
        std::string & error);

    std::uint32_t getBaudRate() const;

    FlowControl getFlowControl() const;

    Parity getParity() const;

    StopBits getStopBits() const;

private:

    std::uint32_t baud_rate_;
    FlowControl flow_control_;
    Parity parity_;
    StopBits stop_bits_;
};

}  // namespace rover::transport::serial

#endif  // ROVER_SERIAL_DRIVER_DOMAIN_SERIAL_PORT_CONFIG_HPP_
