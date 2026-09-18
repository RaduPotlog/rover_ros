// Modbus for c++ <https://github.com/Mazurel/Modbus>
// Copyright (c) 2020 Mateusz Mazur aka Mazurel
// Licensed under: MIT License <http://opensource.org/licenses/MIT>

#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <cerrno>
// Modified 2026 by Mechatronics Academy: dropped <libnet.h>. No libnet_* symbol is used
// anywhere in this library and the target never linked -lnet; the include existed only
// here and in server.hpp, and it cost both Dockerfiles and the README an unnecessary
// libnet1-dev install. It was, however, what transitively supplied ::close() and
// ::inet_addr(), so <unistd.h> and <arpa/inet.h> are now included explicitly.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "modbusException.hpp"
#include "modbusRequest.hpp"
#include "modbusResponse.hpp"

namespace MB::TCP {
class Connection {
  public:
    static const unsigned int DefaultTCPTimeout = 500;
    // Server-side waits are open-ended by nature: a client may take any amount of time to
    // send its next request. Kept at the 60 s the original code hardcoded inline.
    static const unsigned int DefaultRequestTimeout = 60 * 1000;

  private:
    int _sockfd          = -1;
    uint16_t _messageID  = 0;
    int _timeout         = Connection::DefaultTCPTimeout;
    int _requestTimeout  = Connection::DefaultRequestTimeout;

    std::vector<uint8_t> readFrame(int timeoutMs);
    std::vector<uint8_t> recvExactly(std::size_t count);

  public:
    explicit Connection() noexcept : _sockfd(-1), _messageID(0) {};
    explicit Connection(int sockfd) noexcept;
    Connection(const Connection &copy) = delete;
    Connection(Connection &&moved) noexcept;
    Connection &operator=(Connection &&other) noexcept {
        if (this == &other)
            return *this;

        if (_sockfd != -1 && _sockfd != other._sockfd)
            ::close(_sockfd);

        _sockfd    = other._sockfd;
        _messageID = other._messageID;
        // Modified 2026 by Mechatronics Academy: carry the timeouts across moves. They
        // were dropped here and in the move constructor, so a connection configured and
        // then moved - which Connection::with() does, since it returns by value - would
        // silently revert to the defaults.
        _timeout        = other._timeout;
        _requestTimeout = other._requestTimeout;

        other._sockfd = -1;

        return *this;
    }

    [[nodiscard]] int getSockfd() const { return _sockfd; }

    static Connection with(std::string addr, int port);

    ~Connection();

    std::vector<uint8_t> sendRequest(const MB::ModbusRequest &req);
    std::vector<uint8_t> sendResponse(const MB::ModbusResponse &res);
    std::vector<uint8_t> sendException(const MB::ModbusException &ex);

    [[nodiscard]] MB::ModbusRequest awaitRequest();
    [[nodiscard]] MB::ModbusResponse awaitResponse();

    [[nodiscard]] std::vector<uint8_t> awaitRawMessage();

    [[nodiscard]] uint16_t getMessageId() const { return _messageID; }

    void setMessageId(uint16_t messageId) { _messageID = messageId; }

    // Modified 2026 by Mechatronics Academy: added. _timeout was private with no setter,
    // so the response timeout was effectively frozen at 500 ms for every consumer.
    [[nodiscard]] int getTimeout() const { return _timeout; }

    void setTimeout(int timeoutMs) { _timeout = timeoutMs; }

    [[nodiscard]] int getRequestTimeout() const { return _requestTimeout; }

    void setRequestTimeout(int timeoutMs) { _requestTimeout = timeoutMs; }
};
} // namespace MB::TCP
