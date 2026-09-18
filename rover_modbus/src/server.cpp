// Modbus for c++ <https://github.com/Mazurel/Modbus>
// Copyright (c) 2020 Mateusz Mazur aka Mazurel
// Licensed under: MIT License <http://opensource.org/licenses/MIT>

#include "server.hpp"

#include <cerrno>

using namespace MB::TCP;

Server::Server(int port) {
    _port     = port;
    _serverfd = socket(AF_INET, SOCK_STREAM, 0);

    if (_serverfd == -1)
        throw std::runtime_error("Cannot create socket, errno = " + std::to_string(errno));

    // Modified 2026 by Mechatronics Academy: these two were
    // `setsockopt(..., new int(1), sizeof(int))` - a heap allocation per call that was
    // never freed, with the return value ignored. Use a stack value and check.
    const int enable = 1;

    if (::setsockopt(_serverfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
        const int err = errno;
        ::close(_serverfd);
        _serverfd = -1;
        throw std::runtime_error("Cannot set SO_REUSEADDR, errno = " + std::to_string(err));
    }

    if (::setsockopt(_serverfd, SOL_SOCKET, SO_REUSEPORT, &enable, sizeof(enable)) < 0) {
        const int err = errno;
        ::close(_serverfd);
        _serverfd = -1;
        throw std::runtime_error("Cannot set SO_REUSEPORT, errno = " + std::to_string(err));
    }

    _server = {};

    _server.sin_family      = AF_INET;
    _server.sin_addr.s_addr = INADDR_ANY;
    _server.sin_port        = ::htons(static_cast<uint16_t>(_port));

    // Modified 2026 by Mechatronics Academy: close the listening socket before throwing.
    // The destructor cannot run for an object whose constructor threw, so every failed
    // bind() leaked _serverfd.
    if (::bind(_serverfd, reinterpret_cast<struct sockaddr *>(&_server), sizeof(_server)) < 0) {
        const int err = errno;
        ::close(_serverfd);
        _serverfd = -1;
        throw std::runtime_error("Cannot bind socket, errno = " + std::to_string(err));
    }

    // Modified 2026 by Mechatronics Academy: ::listen()'s return value was ignored, so a
    // Server that never entered the listening state looked successfully constructed.
    if (::listen(_serverfd, 255) < 0) {
        const int err = errno;
        ::close(_serverfd);
        _serverfd = -1;
        throw std::runtime_error("Cannot listen on socket, errno = " + std::to_string(err));
    }
}

Server::~Server() {
    if (_serverfd >= 0)
        ::close(_serverfd);

    _serverfd = -1;
}

std::optional<Connection> Server::awaitConnection() {
    socklen_t addrLen = sizeof(_server);

    auto connfd =
        ::accept(_serverfd, reinterpret_cast<struct sockaddr *>(&_server), &addrLen);

    // Modified 2026 by Mechatronics Academy: this was a bare `throw;` outside any catch
    // block, which with no active exception calls std::terminate() and takes the whole
    // process down. The return type is already optional, so report the failure that way.
    if (connfd < 0)
        return std::nullopt;

    return Connection(connfd);
}
