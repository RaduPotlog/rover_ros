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

#include "rover_gps/infrastructure/udp_nmea_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace rover_gps::infrastructure
{

namespace
{

std::string systemError(const char * what, int code)
{
    return std::string(what) + ": " + std::strerror(code);
}

}  // namespace

UdpNmeaReceiver::~UdpNmeaReceiver()
{
    close();
}

bool UdpNmeaReceiver::open(
    const std::string & ip, uint16_t port, double timeout_s, std::size_t buffer_size,
    std::string & error)
{
    close();

    if (buffer_size == 0) {
        error = "buffer_size must be greater than zero";
        return false;
    }

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        error = systemError("socket()", errno);
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
        ::close(fd);
        error = "not a valid IPv4 address: " + ip;
        return false;
    }

    if (::bind(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
        const int code = errno;
        ::close(fd);
        error = systemError("bind()", code);
        return false;
    }

    // A receive timeout is what lets the receive thread notice a shutdown request on a quiet link.
    if (timeout_s >= 1.0e-3) {
        timeval timeout{};
        timeout.tv_sec = static_cast<time_t>(timeout_s);
        timeout.tv_usec = static_cast<suseconds_t>((timeout_s - static_cast<double>(timeout.tv_sec)) * 1.0e6);
        if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            const int code = errno;
            ::close(fd);
            error = systemError("setsockopt(SO_RCVTIMEO)", code);
            return false;
        }
    }

    // Report the port that was actually assigned, so binding port 0 is usable.
    sockaddr_in bound{};
    socklen_t bound_length = sizeof(bound);
    if (::getsockname(fd, reinterpret_cast<sockaddr *>(&bound), &bound_length) == 0) {
        bound_port_ = ntohs(bound.sin_port);
    } else {
        bound_port_ = port;
    }

    buffer_.assign(buffer_size, '\0');
    fd_ = fd;
    return true;
}

ReceiveStatus UdpNmeaReceiver::receive(std::string & payload, std::string & error)
{
    if (fd_ < 0) {
        error = "socket is not open";
        return ReceiveStatus::Error;
    }

    const ssize_t received = ::recvfrom(fd_, buffer_.data(), buffer_.size(), 0, nullptr, nullptr);
    if (received >= 0) {
        payload.assign(buffer_.data(), static_cast<std::size_t>(received));
        return ReceiveStatus::Datagram;
    }

    const int code = errno;
    if (code == EAGAIN || code == EWOULDBLOCK) {
        return ReceiveStatus::Timeout;
    }
    if (code == EINTR) {
        // A signal interrupted the wait; treat it like a timeout so the caller re-checks its flags.
        return ReceiveStatus::Timeout;
    }

    error = systemError("recvfrom()", code);
    return ReceiveStatus::Error;
}

void UdpNmeaReceiver::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    bound_port_ = 0;
}

}  // namespace rover_gps::infrastructure
