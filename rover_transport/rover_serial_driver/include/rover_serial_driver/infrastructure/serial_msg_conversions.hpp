// Copyright 2021 LeoDrive.
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
// Developed by LeoDrive, 2021
//
// Modified 2026 by Mechatronics Academy: the two live UInt8MultiArray converters from
// io_context/msg_converters/std_msgs.hpp (ros-drivers/transport_drivers v1.2.0). The Int*,
// UInt* and Float* declarations in that header were dead, and their definitions in
// std_msgs.cpp dereferenced a uint8_t value as a pointer; both were dropped.

#ifndef ROVER_SERIAL_DRIVER_INFRASTRUCTURE_SERIAL_MSG_CONVERSIONS_HPP_
#define ROVER_SERIAL_DRIVER_INFRASTRUCTURE_SERIAL_MSG_CONVERSIONS_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace rover::transport::serial
{

// `length` is the number of valid bytes in `in`, which is a fixed-size receive buffer.
void toMsg(
    const std::vector<uint8_t> & in,
    std_msgs::msg::UInt8MultiArray & out,
    std::size_t length);

void fromMsg(
    const std_msgs::msg::UInt8MultiArray::SharedPtr & in,
    std::vector<uint8_t> & out);

}  // namespace rover::transport::serial

#endif  // ROVER_SERIAL_DRIVER_INFRASTRUCTURE_SERIAL_MSG_CONVERSIONS_HPP_
