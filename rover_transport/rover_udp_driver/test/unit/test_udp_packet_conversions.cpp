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

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "rover_udp_driver/infrastructure/udp_packet_conversions.hpp"

using rover::transport::udp::fromMsg;
using rover::transport::udp::toMsg;

TEST(UdpPacketConversionsTest, ToMsgCopiesOnlyTheValidPrefix)
{
    const std::vector<uint8_t> buffer{1, 2, 3, 4, 5, 6};
    udp_msgs::msg::UdpPacket msg;

    toMsg(buffer, msg, 4);

    EXPECT_EQ(msg.data, (std::vector<uint8_t>{1, 2, 3, 4}));
}

TEST(UdpPacketConversionsTest, ToMsgClampsToTheBufferSize)
{
    const std::vector<uint8_t> buffer{7};
    udp_msgs::msg::UdpPacket msg;

    toMsg(buffer, msg, 99);

    EXPECT_EQ(msg.data, (std::vector<uint8_t>{7}));
}

TEST(UdpPacketConversionsTest, FromMsgRoundTrips)
{
    auto msg = std::make_shared<udp_msgs::msg::UdpPacket>();
    msg->data = {5, 6, 7, 8};
    std::vector<uint8_t> out;

    fromMsg(msg, out);

    EXPECT_EQ(out, (std::vector<uint8_t>{5, 6, 7, 8}));
}
