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

#include "rover_udp_driver/domain/udp_endpoint.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace rover::transport::udp
{

UdpEndpoint::UdpEndpoint(std::string ip, std::uint16_t port)
: ip_{std::move(ip)},
  port_{port}
{
}

std::optional<UdpEndpoint> UdpEndpoint::fromParameters(
    std::string_view ip,
    int port,
    std::string & error)
{
    if (port <= 0 || port > 65535) {
        error = "port must be in 1..65535, got " + std::to_string(port);
        return std::nullopt;
    }

    return UdpEndpoint{std::string{ip}, static_cast<std::uint16_t>(port)};
}

const std::string & UdpEndpoint::ip() const
{
    return ip_;
}

std::uint16_t UdpEndpoint::port() const
{
    return port_;
}

}  // namespace rover::transport::udp
