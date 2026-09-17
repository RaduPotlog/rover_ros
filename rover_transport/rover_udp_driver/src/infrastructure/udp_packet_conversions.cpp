// Copyright 2021 Evan Flynn.
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
// Developed by Evan Flynn, 2021
//
// Modified 2026 by Mechatronics Academy: moved out of io_context and de-inlined.

#include "rover_udp_driver/infrastructure/udp_packet_conversions.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace rover::transport::udp
{

void toMsg(
    const std::vector<uint8_t> & in,
    udp_msgs::msg::UdpPacket & out,
    std::size_t length)
{
    const std::size_t copied = std::min(length, in.size());
    out.data.resize(copied);
    std::copy(in.begin(), in.begin() + static_cast<std::ptrdiff_t>(copied), out.data.begin());
}

void fromMsg(
    const udp_msgs::msg::UdpPacket::SharedPtr & in,
    std::vector<uint8_t> & out)
{
    out.resize(in->data.size());
    std::copy(in->data.begin(), in->data.end(), out.begin());
}

}  // namespace rover::transport::udp
