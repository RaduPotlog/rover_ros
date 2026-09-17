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

#include <string>

#include "rover_udp_driver/domain/udp_endpoint.hpp"

using rover::transport::udp::UdpEndpoint;

TEST(UdpEndpointTest, AcceptsAValidAddressAndPort)
{
    std::string error;
    const auto endpoint = UdpEndpoint::fromParameters("192.168.1.201", 4444, error);

    ASSERT_TRUE(endpoint.has_value());
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(endpoint->ip(), "192.168.1.201");
    EXPECT_EQ(endpoint->port(), 4444);
}

TEST(UdpEndpointTest, AcceptsAnEmptyAddressMeaningAny)
{
    // Upstream's UdpSocket treats an empty ip as udp::v4() - keep that behaviour.
    std::string error;
    const auto endpoint = UdpEndpoint::fromParameters("", 3333, error);

    ASSERT_TRUE(endpoint.has_value());
    EXPECT_TRUE(endpoint->ip().empty());
}

TEST(UdpEndpointTest, RejectsAPortOutsideTheValidRange)
{
    // Upstream defaulted port to 0 and never checked it, so a missing parameter surfaced
    // as an ASIO bind error instead of a configuration failure.
    std::string error;
    EXPECT_FALSE(UdpEndpoint::fromParameters("127.0.0.1", 0, error).has_value());
    EXPECT_NE(error.find("port"), std::string::npos);

    error.clear();
    EXPECT_FALSE(UdpEndpoint::fromParameters("127.0.0.1", -1, error).has_value());
    EXPECT_FALSE(error.empty());

    error.clear();
    EXPECT_FALSE(UdpEndpoint::fromParameters("127.0.0.1", 65536, error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(UdpEndpointTest, AcceptsTheBoundaryPorts)
{
    std::string error;
    EXPECT_TRUE(UdpEndpoint::fromParameters("127.0.0.1", 1, error).has_value());
    EXPECT_TRUE(UdpEndpoint::fromParameters("127.0.0.1", 65535, error).has_value());
}
