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
// Modified 2026 by Mechatronics Academy: relayouted from udp_driver/src/udp_socket.cpp
// (ros-drivers/transport_drivers v1.2.0). Three defects fixed while moving - see
// asyncSend(), asyncReceiveHandler() and send()/receive().

#include "rover_udp_driver/infrastructure/asio_udp_socket.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>

namespace rover::transport::udp
{

namespace
{

asio::ip::udp::endpoint makeEndpoint(const std::string & ip, std::uint16_t port)
{
    return ip.empty() ?
           asio::ip::udp::endpoint{asio::ip::udp::v4(), port} :
           asio::ip::udp::endpoint{asio::ip::address::from_string(ip), port};
}

}  // namespace

AsioUdpSocket::AsioUdpSocket(
    const IoContext & ctx,
    const std::string & remote_ip,
    const std::uint16_t remote_port,
    const std::string & host_ip,
    const std::uint16_t host_port,
    SocketRole role)
: ctx_(ctx),
  udp_socket_(ctx.ios()),
  remote_endpoint_(makeEndpoint(remote_ip, remote_port)),
  host_endpoint_(makeEndpoint(host_ip, host_port)),
  role_(role)
{
    recv_buffer_.resize(kRecvBufferSize);
}

AsioUdpSocket::AsioUdpSocket(
    const IoContext & ctx,
    const std::string & ip,
    const std::uint16_t port,
    SocketRole role)
: AsioUdpSocket{ctx, ip, port, ip, port, role}
{
}

AsioUdpSocket::~AsioUdpSocket()
{
    close();
}

std::size_t AsioUdpSocket::send(const std::vector<uint8_t> & buffer)
{
    try {
        return udp_socket_.send_to(asio::buffer(buffer), remote_endpoint_);
    } catch (const std::system_error & error) {
        // Upstream returned -1 from a std::size_t function, which wraps to SIZE_MAX and
        // reads as a huge successful send. 0 is the honest answer.
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AsioUdpSocket::send"), error.what());
        return 0;
    }
}

std::size_t AsioUdpSocket::receive(std::vector<uint8_t> & buffer)
{
    asio::error_code error;
    const std::size_t len = udp_socket_.receive_from(
        asio::buffer(buffer), host_endpoint_, 0, error);

    if (error && error != asio::error::message_size) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AsioUdpSocket::receive"), error.message());
        return 0;
    }
    return len;
}

void AsioUdpSocket::asyncSend(const std::vector<uint8_t> & buffer)
{
    // Upstream held asio::buffer() over a caller-owned vector that died when the
    // subscriber callback returned, leaving the async write reading freed memory.
    auto payload = std::make_shared<std::vector<uint8_t>>(buffer);

    udp_socket_.async_send_to(
        asio::buffer(*payload), remote_endpoint_,
        [this, payload](std::error_code error, std::size_t bytes_transferred)
        {
            asyncSendHandler(error, bytes_transferred);
        });
}

void AsioUdpSocket::asyncReceive(ByteReceiveCallback callback)
{
    callback_ = std::move(callback);
    armReceive();
}

void AsioUdpSocket::armReceive()
{
    udp_socket_.async_receive_from(
        asio::buffer(recv_buffer_),
        host_endpoint_,
        [this](std::error_code error, std::size_t bytes_transferred)
        {
            asyncReceiveHandler(error, bytes_transferred);
        });
}

void AsioUdpSocket::asyncSendHandler(
    const asio::error_code & error,
    std::size_t bytes_transferred)
{
    (void)bytes_transferred;
    if (error) {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("AsioUdpSocket::asyncSendHandler"), error.message());
    }
}

void AsioUdpSocket::asyncReceiveHandler(
    const asio::error_code & error,
    std::size_t bytes_transferred)
{
    if (error) {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("AsioUdpSocket::asyncReceiveHandler"), error.message());
        return;
    }

    if (bytes_transferred > 0 && callback_) {
        // Upstream resized the receive buffer down to the datagram length, handed it to the
        // callback, resized it back up, and then re-armed with a lambda that resized it
        // down again *before* dispatching - so the next read could land in a 2-byte buffer.
        // Passing (buffer, length) instead means the buffer keeps its full capacity.
        callback_(recv_buffer_, bytes_transferred);
    }

    // Re-arm unconditionally on success: a zero-length datagram is legal UDP and must not
    // silently stop the receiver, which is what upstream's `if` did.
    armReceive();
}

std::string AsioUdpSocket::remoteIp() const
{
    return remote_endpoint_.address().to_string();
}

std::uint16_t AsioUdpSocket::remotePort() const
{
    return remote_endpoint_.port();
}

std::string AsioUdpSocket::hostIp() const
{
    return host_endpoint_.address().to_string();
}

std::uint16_t AsioUdpSocket::hostPort() const
{
    return host_endpoint_.port();
}

void AsioUdpSocket::open()
{
    udp_socket_.open(asio::ip::udp::v4());
    udp_socket_.set_option(asio::ip::udp::socket::reuse_address(true));

    if (role_ == SocketRole::RECEIVER) {
        udp_socket_.bind(host_endpoint_);
    }
}

void AsioUdpSocket::close()
{
    asio::error_code error;
    udp_socket_.close(error);
    if (error) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("AsioUdpSocket::close"), error.message());
    }
}

bool AsioUdpSocket::isOpen() const
{
    return udp_socket_.is_open();
}

std::unique_ptr<ByteStreamPort> makeUdpReceiver(
    const IoContext & ctx,
    const UdpEndpoint & endpoint)
{
    return std::make_unique<AsioUdpSocket>(
        ctx, endpoint.ip(), endpoint.port(), SocketRole::RECEIVER);
}

std::unique_ptr<ByteStreamPort> makeUdpSender(
    const IoContext & ctx,
    const UdpEndpoint & endpoint)
{
    return std::make_unique<AsioUdpSocket>(
        ctx, endpoint.ip(), endpoint.port(), SocketRole::SENDER);
}

}  // namespace rover::transport::udp
