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
// Modified 2026 by Mechatronics Academy: merges udp_driver's test_udp_socket.cpp,
// test_udp_data.cpp and test_udp_driver.cpp (ros-drivers/transport_drivers v1.2.0). All
// three hardcoded 127.0.0.1:8000, so they could not run concurrently; each case here
// takes a distinct port derived from the pid, which also keeps parallel colcon runs on
// one machine from colliding.

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <unistd.h>

#include "rover_udp_driver/infrastructure/asio_udp_socket.hpp"

using rover::transport::IoContext;
using rover::transport::udp::AsioUdpSocket;
using rover::transport::udp::SocketRole;

namespace
{

constexpr const char * kIp = "127.0.0.1";

// Ports in the ephemeral-ish range, unique per process and per case.
std::uint16_t testPort(std::uint16_t offset)
{
    return static_cast<std::uint16_t>(20000 + (getpid() % 10000) + offset);
}

}  // namespace

TEST(AsioUdpSocketTest, LifeCycleTest)
{
    IoContext ctx;
    const auto port = testPort(0);
    AsioUdpSocket socket(ctx, kIp, port, SocketRole::SENDER);

    EXPECT_EQ(socket.remoteIp(), kIp);
    EXPECT_EQ(socket.remotePort(), port);

    EXPECT_FALSE(socket.isOpen());
    socket.open();
    EXPECT_TRUE(socket.isOpen());
    socket.close();
    EXPECT_FALSE(socket.isOpen());

    ctx.waitForExit();
}

TEST(AsioUdpSocketTest, ReceiverBindsOnOpen)
{
    IoContext ctx;
    AsioUdpSocket socket(ctx, kIp, testPort(1), SocketRole::RECEIVER);

    // Upstream required a separate bind() call; a receiver that forgot it silently never
    // received anything.
    EXPECT_NO_THROW(socket.open());
    EXPECT_TRUE(socket.isOpen());
    socket.close();

    ctx.waitForExit();
}

TEST(AsioUdpSocketTest, AsyncSendReachesAnAsyncReceiver)
{
    IoContext ctx;
    const auto port = testPort(2);

    AsioUdpSocket receiver(ctx, kIp, port, SocketRole::RECEIVER);
    AsioUdpSocket sender(ctx, kIp, port, SocketRole::SENDER);

    std::mutex mutex;
    std::condition_variable cv;
    std::vector<uint8_t> received;
    bool got_it = false;

    receiver.open();
    receiver.asyncReceive(
        [&](const std::vector<uint8_t> & buffer, std::size_t length)
        {
            std::lock_guard<std::mutex> lock{mutex};
            received.assign(buffer.begin(), buffer.begin() + static_cast<long>(length));
            got_it = true;
            cv.notify_one();
        });

    sender.open();
    // The payload is a temporary here on purpose: upstream held an asio::buffer over the
    // caller's vector, so this is exactly the shape that used to read freed memory.
    sender.asyncSend(std::vector<uint8_t>{1, 2, 3, 4});

    {
        std::unique_lock<std::mutex> lock{mutex};
        ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(5), [&] {return got_it;}));
    }

    EXPECT_EQ(received, (std::vector<uint8_t>{1, 2, 3, 4}));

    sender.close();
    receiver.close();
    ctx.waitForExit();
}

TEST(AsioUdpSocketTest, BlockingSendReportsZeroOnAClosedSocket)
{
    IoContext ctx;
    AsioUdpSocket socket(ctx, kIp, testPort(3), SocketRole::SENDER);

    const std::vector<uint8_t> payload{1, 2, 3};

    // Upstream returned -1 from a std::size_t function, which wraps to SIZE_MAX and reads
    // as an enormous successful send.
    EXPECT_EQ(socket.send(payload), 0u);

    ctx.waitForExit();
}
