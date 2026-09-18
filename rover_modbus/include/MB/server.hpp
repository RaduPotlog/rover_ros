// Modbus for c++ <https://github.com/Mazurel/Modbus>
// Copyright (c) 2020 Mateusz Mazur aka Mazurel
// Licensed under: MIT License <http://opensource.org/licenses/MIT>

#pragma once

#include <optional>
#include <stdexcept>
#include <string>

// Modified 2026 by Mechatronics Academy: dropped <libnet.h> (unused, never linked); it
// was transitively supplying ::close(), so <unistd.h> is now explicit.
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "connection.hpp"

namespace MB::TCP {
class Server {
  private:
    int _serverfd;
    int _port;
    sockaddr_in _server;

  public:
    explicit Server(int port);
    ~Server();

    Server(const Server &) = delete;
    Server(Server &&moved) {
        _serverfd = moved._serverfd;
        _port     = moved._port;
        // Modified 2026 by Mechatronics Academy: carry _server across moves. It was
        // dropped by both move operations, leaving awaitConnection() to hand an
        // uninitialised sockaddr_in to ::accept() on any moved-to Server.
        _server         = moved._server;
        moved._serverfd = -1;
    }
    Server &operator=(Server &&moved) {
        if (this == &moved)
            return *this;

        if (_serverfd >= 0 && _serverfd != moved._serverfd)
            ::close(_serverfd);

        _serverfd       = moved._serverfd;
        _port           = moved._port;
        _server         = moved._server;
        moved._serverfd = -1;
        return *this;
    }

    [[nodiscard]] int nativeHandle() { return _serverfd; }

    [[nodiscard]] int port() const { return _port; }

    std::optional<Connection> awaitConnection();
};
} // namespace MB::TCP
