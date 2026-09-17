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

#ifndef ROVER_GPS_INFRASTRUCTURE_UDP_NMEA_RECEIVER_HPP_
#define ROVER_GPS_INFRASTRUCTURE_UDP_NMEA_RECEIVER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rover_gps::infrastructure
{

/** @brief Outcome of one @ref UdpNmeaReceiver::receive call. */
enum class ReceiveStatus
{
    Datagram,  // a datagram was read into the output buffer
    Timeout,   // nothing arrived within the configured timeout - the link is merely quiet
    Error,     // the socket failed and should be closed and re-opened
};

/**
 * @brief Blocking UDP listener for NMEA datagrams.
 * @details Owns a bound datagram socket and closes it on destruction. `receive()` blocks for at
 *          most the timeout given to `open()`, which is what lets a caller poll a shutdown flag
 *          on a quiet link instead of hanging until the next datagram arrives.
 *          Not thread-safe: one thread at a time.
 */
class UdpNmeaReceiver
{
public:
    UdpNmeaReceiver() = default;
    ~UdpNmeaReceiver();

    UdpNmeaReceiver(const UdpNmeaReceiver &) = delete;
    UdpNmeaReceiver & operator=(const UdpNmeaReceiver &) = delete;

    /**
     * @brief Creates and binds the socket.
     * @param ip          Local address to bind, e.g. "0.0.0.0" for every interface.
     * @param port        Local UDP port; 0 binds an ephemeral port, which `boundPort()` reports.
     * @param timeout_s   Receive timeout [s]; values below one millisecond are treated as blocking.
     * @param buffer_size Maximum datagram size to accept [bytes].
     * @param[out] error  Human-readable reason when this returns false.
     */
    bool open(
        const std::string & ip, uint16_t port, double timeout_s, std::size_t buffer_size,
        std::string & error);

    /**
     * @brief Reads one datagram.
     * @param[out] payload Datagram contents; untouched unless the result is `Datagram`.
     * @param[out] error   Human-readable reason when the result is `Error`.
     */
    ReceiveStatus receive(std::string & payload, std::string & error);

    void close();

    bool isOpen() const { return fd_ >= 0; }

    /** @brief The port actually bound, which differs from the requested one when 0 was asked for. */
    uint16_t boundPort() const { return bound_port_; }

private:
    int fd_{-1};
    uint16_t bound_port_{0};
    std::vector<char> buffer_;
};

}  // namespace rover_gps::infrastructure

#endif  // ROVER_GPS_INFRASTRUCTURE_UDP_NMEA_RECEIVER_HPP_
