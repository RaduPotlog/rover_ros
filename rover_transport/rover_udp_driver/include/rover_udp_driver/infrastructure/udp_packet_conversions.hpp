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
// Modified 2026 by Mechatronics Academy: moved out of
// io_context/include/msg_converters/udp_msgs.hpp (ros-drivers/transport_drivers v1.2.0)
// and de-inlined, so rover_io_context no longer depends on udp_msgs.

#ifndef ROVER_UDP_DRIVER_INFRASTRUCTURE_UDP_PACKET_CONVERSIONS_HPP_
#define ROVER_UDP_DRIVER_INFRASTRUCTURE_UDP_PACKET_CONVERSIONS_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <udp_msgs/msg/udp_packet.hpp>

namespace rover::transport::udp
{

// `length` is the number of valid bytes in `in`, which is a fixed-size receive buffer.
void toMsg(
    const std::vector<uint8_t> & in,
    udp_msgs::msg::UdpPacket & out,
    std::size_t length);

void fromMsg(
    const udp_msgs::msg::UdpPacket::SharedPtr & in,
    std::vector<uint8_t> & out);

}  // namespace rover::transport::udp

#endif  // ROVER_UDP_DRIVER_INFRASTRUCTURE_UDP_PACKET_CONVERSIONS_HPP_
