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
// Modified 2026 by Mechatronics Academy: UdpSocket -> AsioUdpSocket implementing
// rover::transport::ByteStreamPort (ros-drivers/transport_drivers v1.2.0). UdpDriver, a
// two-member shared_ptr holder, was dropped in favour of the factories below.

#ifndef ROVER_UDP_DRIVER_INFRASTRUCTURE_ASIO_UDP_SOCKET_HPP_
#define ROVER_UDP_DRIVER_INFRASTRUCTURE_ASIO_UDP_SOCKET_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <asio.hpp>

#include "rover_io_context/domain/ports.hpp"
#include "rover_io_context/infrastructure/io_context.hpp"
#include "rover_udp_driver/domain/udp_endpoint.hpp"

namespace rover::transport::udp
{

// Whether open() should also bind. A receiver must bind to its host endpoint; a sender
// must not. Upstream exposed bind() as a separate public call the node had to remember to
// make, which is how it ended up on the ByteStreamPort's wrong side.
enum class SocketRole
{
    SENDER,
    RECEIVER
};

class AsioUdpSocket : public ByteStreamPort
{

public:

    AsioUdpSocket(
        const IoContext & ctx,
        const std::string & remote_ip, std::uint16_t remote_port,
        const std::string & host_ip, std::uint16_t host_port,
        SocketRole role);

    AsioUdpSocket(
        const IoContext & ctx,
        const std::string & ip, std::uint16_t port,
        SocketRole role);

    ~AsioUdpSocket() override;

    AsioUdpSocket(const AsioUdpSocket &) = delete;
    AsioUdpSocket & operator=(const AsioUdpSocket &) = delete;

    std::string remoteIp() const;
    std::uint16_t remotePort() const;
    std::string hostIp() const;
    std::uint16_t hostPort() const;

    // ByteStreamPort. For a RECEIVER, open() also binds.
    void open() override;
    void close() override;
    bool isOpen() const override;
    void asyncSend(const std::vector<uint8_t> & buffer) override;
    void asyncReceive(ByteReceiveCallback callback) override;

    // Blocking operations, kept off ByteStreamPort: used only by the tests.
    std::size_t send(const std::vector<uint8_t> & buffer);

    std::size_t receive(std::vector<uint8_t> & buffer);

private:

    void asyncSendHandler(const asio::error_code & error, std::size_t bytes_transferred);

    void asyncReceiveHandler(const asio::error_code & error, std::size_t bytes_transferred);

    void armReceive();

    const IoContext & ctx_;
    asio::ip::udp::socket udp_socket_;
    asio::ip::udp::endpoint remote_endpoint_;
    asio::ip::udp::endpoint host_endpoint_;
    SocketRole role_;
    ByteReceiveCallback callback_;
    static constexpr std::size_t kRecvBufferSize{2048};
    std::vector<uint8_t> recv_buffer_;
};

std::unique_ptr<ByteStreamPort> makeUdpReceiver(
    const IoContext & ctx,
    const UdpEndpoint & endpoint);

std::unique_ptr<ByteStreamPort> makeUdpSender(
    const IoContext & ctx,
    const UdpEndpoint & endpoint);

}  // namespace rover::transport::udp

#endif  // ROVER_UDP_DRIVER_INFRASTRUCTURE_ASIO_UDP_SOCKET_HPP_
