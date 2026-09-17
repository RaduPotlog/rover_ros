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

#include "rover_serial_driver/infrastructure/serial_msg_conversions.hpp"

using rover::transport::serial::fromMsg;
using rover::transport::serial::toMsg;

TEST(SerialMsgConversionsTest, ToMsgCopiesOnlyTheValidPrefix)
{
    const std::vector<uint8_t> buffer{1, 2, 3, 4, 5, 6, 7, 8};
    std_msgs::msg::UInt8MultiArray msg;

    toMsg(buffer, msg, 3);

    EXPECT_EQ(msg.data, (std::vector<uint8_t>{1, 2, 3}));
}

TEST(SerialMsgConversionsTest, ToMsgClampsToTheBufferSize)
{
    // Upstream memcpy'd `bytes_transferred` bytes unconditionally; a length longer than
    // the buffer would have read out of bounds.
    const std::vector<uint8_t> buffer{9, 8};
    std_msgs::msg::UInt8MultiArray msg;

    toMsg(buffer, msg, 16);

    EXPECT_EQ(msg.data, (std::vector<uint8_t>{9, 8}));
}

TEST(SerialMsgConversionsTest, ToMsgHandlesAnEmptyRead)
{
    const std::vector<uint8_t> buffer{1, 2, 3};
    std_msgs::msg::UInt8MultiArray msg;

    toMsg(buffer, msg, 0);

    EXPECT_TRUE(msg.data.empty());
}

TEST(SerialMsgConversionsTest, FromMsgRoundTrips)
{
    auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
    msg->data = {10, 20, 30};
    std::vector<uint8_t> out;

    fromMsg(msg, out);

    EXPECT_EQ(out, (std::vector<uint8_t>{10, 20, 30}));
}
