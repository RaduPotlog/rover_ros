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

#ifndef ROVER_UDP_DRIVER_DOMAIN_UDP_ENDPOINT_HPP_
#define ROVER_UDP_DRIVER_DOMAIN_UDP_ENDPOINT_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace rover::transport::udp
{

// An IP address and port, validated. Upstream read both parameters straight into the node
// and never checked them, so port 0 or a malformed address only failed later inside ASIO.
class UdpEndpoint
{

public:

    UdpEndpoint(std::string ip, std::uint16_t port);

    // An empty ip is accepted and means "any address" (udp::v4()), matching upstream's
    // behaviour in UdpSocket. A port outside 1-65535 is rejected.
    static std::optional<UdpEndpoint> fromParameters(
        std::string_view ip,
        int port,
        std::string & error);

    const std::string & ip() const;

    std::uint16_t port() const;

private:

    std::string ip_;
    std::uint16_t port_;
};

}  // namespace rover::transport::udp

#endif  // ROVER_UDP_DRIVER_DOMAIN_UDP_ENDPOINT_HPP_
