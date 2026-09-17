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
// Modified 2026 by Mechatronics Academy: extracted from
// io_context/include/msg_converters/std_msgs.hpp (ros-drivers/transport_drivers v1.2.0).

#include "rover_serial_driver/infrastructure/serial_msg_conversions.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace rover::transport::serial
{

void toMsg(
    const std::vector<uint8_t> & in,
    std_msgs::msg::UInt8MultiArray & out,
    std::size_t length)
{
    const std::size_t copied = std::min(length, in.size());
    out.data.resize(copied);
    std::copy(in.begin(), in.begin() + static_cast<std::ptrdiff_t>(copied), out.data.begin());
}

void fromMsg(
    const std_msgs::msg::UInt8MultiArray::SharedPtr & in,
    std::vector<uint8_t> & out)
{
    out = in->data;
}

}  // namespace rover::transport::serial
