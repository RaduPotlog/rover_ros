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

// Covers the behaviour that upstream had duplicated inline in three lifecycle nodes and
// could not test without a ROS graph: the receive->publish path, and the rule that a
// deactivated bridge must not write to the hardware.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "rover_io_context/application/inbound_byte_bridge.hpp"
#include "rover_io_context/application/outbound_byte_bridge.hpp"
#include "rover_io_context/domain/ports.hpp"

using rover::transport::ByteReceiveCallback;
using rover::transport::BytePublisherPort;
using rover::transport::ByteStreamPort;
using rover::transport::InboundByteBridge;
using rover::transport::OutboundByteBridge;

namespace
{

class FakeStream : public ByteStreamPort
{

public:

    void open() override {open_ = true;}

    void close() override {open_ = false;}

    bool isOpen() const override {return open_;}

    void asyncSend(const std::vector<uint8_t> & buffer) override
    {
        sent_.push_back(buffer);
    }

    void asyncReceive(ByteReceiveCallback callback) override
    {
        callback_ = std::move(callback);
    }

    // Drives the registered callback as a real stream would.
    void deliver(const std::vector<uint8_t> & buffer, std::size_t length)
    {
        if (callback_) {
            callback_(buffer, length);
        }
    }

    bool hasCallback() const {return static_cast<bool>(callback_);}

    const std::vector<std::vector<uint8_t>> & sent() const {return sent_;}

private:

    bool open_{false};
    ByteReceiveCallback callback_;
    std::vector<std::vector<uint8_t>> sent_;
};

class FakePublisher : public BytePublisherPort
{

public:

    void publish(const std::vector<uint8_t> & buffer, std::size_t length) override
    {
        published_.emplace_back(buffer.begin(), buffer.begin() + static_cast<long>(length));
    }

    const std::vector<std::vector<uint8_t>> & published() const {return published_;}

private:

    std::vector<std::vector<uint8_t>> published_;
};

}  // namespace

TEST(InboundByteBridgeTest, StartArmsTheStream)
{
    FakeStream stream;
    FakePublisher publisher;
    InboundByteBridge bridge{stream, publisher};

    EXPECT_FALSE(stream.hasCallback());
    bridge.start();
    EXPECT_TRUE(stream.hasCallback());
}

TEST(InboundByteBridgeTest, ForwardsOnlyTheValidPrefixOfTheBuffer)
{
    FakeStream stream;
    FakePublisher publisher;
    InboundByteBridge bridge{stream, publisher};
    bridge.start();

    // A serial receive buffer is a fixed-size scratch buffer: 8 bytes wide, 3 bytes valid.
    stream.deliver({1, 2, 3, 0, 0, 0, 0, 0}, 3);

    ASSERT_EQ(publisher.published().size(), 1u);
    EXPECT_EQ(publisher.published().front(), (std::vector<uint8_t>{1, 2, 3}));
    EXPECT_EQ(bridge.bytesForwarded(), 3u);
}

TEST(InboundByteBridgeTest, DropsEmptyReads)
{
    FakeStream stream;
    FakePublisher publisher;
    InboundByteBridge bridge{stream, publisher};
    bridge.start();

    stream.deliver({0, 0, 0}, 0);

    EXPECT_TRUE(publisher.published().empty());
    EXPECT_EQ(bridge.bytesForwarded(), 0u);
}

TEST(OutboundByteBridgeTest, DropsEverythingWhileInactive)
{
    FakeStream stream;
    OutboundByteBridge bridge{stream};

    // Default state is inactive - a freshly constructed bridge must not drive hardware.
    EXPECT_FALSE(bridge.isActive());
    bridge.send({1, 2, 3});

    EXPECT_TRUE(stream.sent().empty());
    EXPECT_EQ(bridge.bytesSent(), 0u);
    EXPECT_EQ(bridge.bytesDropped(), 3u);
}

TEST(OutboundByteBridgeTest, SendsWhileActive)
{
    FakeStream stream;
    OutboundByteBridge bridge{stream};
    bridge.setActive(true);

    bridge.send({4, 5});

    ASSERT_EQ(stream.sent().size(), 1u);
    EXPECT_EQ(stream.sent().front(), (std::vector<uint8_t>{4, 5}));
    EXPECT_EQ(bridge.bytesSent(), 2u);
    EXPECT_EQ(bridge.bytesDropped(), 0u);
}

TEST(OutboundByteBridgeTest, StopsSendingAfterDeactivation)
{
    FakeStream stream;
    OutboundByteBridge bridge{stream};

    bridge.setActive(true);
    bridge.send({1});
    bridge.setActive(false);
    bridge.send({2, 3});

    ASSERT_EQ(stream.sent().size(), 1u);
    EXPECT_EQ(stream.sent().front(), (std::vector<uint8_t>{1}));
    EXPECT_EQ(bridge.bytesSent(), 1u);
    EXPECT_EQ(bridge.bytesDropped(), 2u);
}
