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

#ifndef ROVER_IO_CONTEXT_DOMAIN_PORTS_HPP_
#define ROVER_IO_CONTEXT_DOMAIN_PORTS_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace rover::transport
{

// Upstream had two incompatible receive signatures - serial passed
// (std::vector<uint8_t> &, const size_t &) and UDP passed (const std::vector<uint8_t> &).
// One shape lets a single pair of bridges serve both transports. The length is explicit
// because the serial receive buffer is a fixed-size scratch buffer that is only partly
// filled; `buffer.size()` is NOT the number of bytes received.
using ByteReceiveCallback =
    std::function<void (const std::vector<uint8_t> & buffer, std::size_t length)>;

// A bidirectional byte stream: a UART or a UDP socket. Implemented in infrastructure by
// AsioSerialPort (rover_serial_driver) and AsioUdpSocket (rover_udp_driver).
//
// Deliberately minimal. The blocking send/receive pair and send_break() are serial-only
// and used by nothing but tests, so they stay concrete methods on AsioSerialPort rather
// than widening this interface for one implementation.
class ByteStreamPort
{

public:

    virtual ~ByteStreamPort() = default;

    // For a UDP receiver, open() also binds - binding is not a separate concept a caller
    // should have to know about.
    virtual void open() = 0;

    virtual void close() = 0;

    virtual bool isOpen() const = 0;

    virtual void asyncSend(const std::vector<uint8_t> & buffer) = 0;

    // Registers the callback and arms the first read. The implementation re-arms itself
    // after every completed read until the stream is closed.
    virtual void asyncReceive(ByteReceiveCallback callback) = 0;
};

// Where received bytes go. Implemented in infrastructure by the lifecycle publishers that
// wrap them in UInt8MultiArray (serial) or UdpPacket (UDP).
class BytePublisherPort
{

public:

    virtual ~BytePublisherPort() = default;

    virtual void publish(const std::vector<uint8_t> & buffer, std::size_t length) = 0;
};

}  // namespace rover::transport

#endif  // ROVER_IO_CONTEXT_DOMAIN_PORTS_HPP_
