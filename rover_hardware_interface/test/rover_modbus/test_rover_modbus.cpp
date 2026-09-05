// Copyright 2025 Mechatronics Academy
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
// Integration tests for RoverModbus - the real Modbus TCP client backed by the vendored
// Modbus_Core library, previously exercised only through FakeRoverModbus (see
// rover_safety_controller/test_rover_safety_controller.cpp and fake_rover_modbus.hpp). No live
// Modbus server is required: these only exercise the connection-establishment path (constructor +
// retry/backoff), which fails fast and deterministically against a closed TCP port on loopback
// (ECONNREFUSED is immediate on Linux - no listener means the kernel sends RST right away, so
// this never waits on the library's internal TCP connect timeout).

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <stdexcept>

#include "rover_hardware_interface/rover_modbus/modbus.hpp"

namespace rover_hardware_interface
{
namespace test
{

namespace
{

// Binds an ephemeral TCP port on loopback, then immediately closes it - for the lifetime of this
// short-lived test the returned port is very likely still closed, so connecting to it fails fast
// with ECONNREFUSED rather than hanging.
int reserveClosedLoopbackPort()
{
    const int sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        throw std::runtime_error("Failed to create probe socket.");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;

    if (::bind(sockfd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        ::close(sockfd);
        throw std::runtime_error("Failed to bind probe socket.");
    }

    socklen_t addr_len = sizeof(addr);
    if (::getsockname(sockfd, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
        ::close(sockfd);
        throw std::runtime_error("Failed to read probe socket's assigned port.");
    }

    const int port = ntohs(addr.sin_port);
    ::close(sockfd);

    return port;
}

}  // namespace

TEST(RoverModbusTest, EmptyHostThrowsInvalidArgument)
{
    EXPECT_THROW(
        RoverModbus("", 502, 1, std::chrono::milliseconds(0)), std::invalid_argument);
}

TEST(RoverModbusTest, UnreachableHostThrowsRuntimeErrorAfterExhaustingRetries)
{
    const int closed_port = reserveClosedLoopbackPort();

    // Bounded: max_connection_attempts=2 with a short retry_delay keeps this test's worst-case
    // runtime well under a second, since a refused loopback connect fails immediately rather than
    // waiting out a connect timeout.
    EXPECT_THROW(
        RoverModbus("127.0.0.1", closed_port, 2, std::chrono::milliseconds(5)),
        std::runtime_error);
}

}  // namespace test
}  // namespace rover_hardware_interface
