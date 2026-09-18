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

//
// Integration tests for the real Modbus TCP client, moved here from
// rover_hardware_interface/test/rover_modbus/test_rover_modbus.cpp along with the code it
// exercises. No live Modbus server is required: these only cover the
// connection-establishment path (factory + retry/backoff), which fails fast and
// deterministically against a closed TCP port on loopback - ECONNREFUSED is immediate on
// Linux, since with no listener the kernel replies RST straight away, so this never waits
// on a TCP connect timeout.
//
// The request/response path is covered separately: see test/unit for the encoding and
// test/e2e for a full round trip against a real MB::TCP::Server.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdexcept>

#include "rover_modbus_driver/infrastructure/modbus_tcp_client_factory.hpp"

namespace rover::transport::modbus::test
{

namespace
{

// Binds an ephemeral TCP port on loopback, then immediately closes it - for the lifetime
// of this short-lived test the returned port is very likely still closed, so connecting to
// it fails fast with ECONNREFUSED rather than hanging.
int reserveClosedLoopbackPort()
{
    const int sockfd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        throw std::runtime_error("Failed to create probe socket.");
    }

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = 0;

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

TEST(ModbusTcpTransportTest, EmptyHostThrowsInvalidArgument)
{
    ClientSettings settings;
    settings.host                      = "";
    settings.port                      = 502;
    settings.connection_retry_count    = 1;
    settings.connection_retry_delay_ms = 0;

    EXPECT_THROW(makeModbusTcpDiscreteIoClient(settings), std::invalid_argument);
}

TEST(ModbusTcpTransportTest, UnreachableHostThrowsRuntimeErrorAfterExhaustingRetries)
{
    ClientSettings settings;
    settings.host = "127.0.0.1";
    settings.port = reserveClosedLoopbackPort();
    // Bounded: 2 attempts with a short delay keeps the worst case well under a second,
    // since a refused loopback connect fails immediately.
    settings.connection_retry_count    = 2;
    settings.connection_retry_delay_ms = 5;

    EXPECT_THROW(makeModbusTcpDiscreteIoClient(settings), std::runtime_error);
}

}  // namespace rover::transport::modbus::test
