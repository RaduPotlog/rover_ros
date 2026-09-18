// Modbus for c++ <https://github.com/Mazurel/Modbus>
// Copyright (c) 2020 Mateusz Mazur aka Mazurel
// Licensed under: MIT License <http://opensource.org/licenses/MIT>

#include "connection.hpp"
#include <cstdint>
#include <cstring>
#include <sys/poll.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace MB::TCP;

namespace {

// Modified 2026 by Mechatronics Academy: the MBAP header was assembled inline three times
// (sendRequest / sendResponse / sendException) with a length encoding that was wrong in
// two ways - it pushed `reinterpret_cast<const uint16_t *>(&size)[1]`, i.e. the HIGH half
// of a uint32_t (always 0 for real frames), then a truncated low half, into a
// vector<uint8_t>. That is endian-dependent and silently wrong for any PDU over 255
// bytes; it only worked because discrete-IO frames are tiny. Factored into one helper
// with an explicit big-endian encode.

void pushBigEndian16(std::vector<uint8_t> &out, const uint16_t value) {
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
    out.push_back(static_cast<uint8_t>(value & 0xFFu));
}

uint16_t readBigEndian16(const uint8_t *const bytes) {
    return static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                                 static_cast<uint16_t>(bytes[1]));
}

// MBAP: transaction id (2) | protocol id (2, always 0) | length (2) | PDU.
// `length` counts the bytes that follow it, i.e. the PDU including the unit id.
std::vector<uint8_t> buildMbapFrame(const uint16_t messageID, const std::vector<uint8_t> &pdu) {
    std::vector<uint8_t> frame;
    frame.reserve(6 + pdu.size());

    pushBigEndian16(frame, messageID);
    pushBigEndian16(frame, 0x0000);                          // protocol identifier
    pushBigEndian16(frame, static_cast<uint16_t>(pdu.size()));

    frame.insert(frame.end(), pdu.begin(), pdu.end());

    return frame;
}

// Modified 2026 by Mechatronics Academy: ::send()'s return value was ignored, so a
// partial write on a full socket buffer emitted a truncated frame with no error. Loop
// until the whole frame is out.
void sendAll(const int sockfd, const std::vector<uint8_t> &frame) {
    std::size_t sent = 0;

    while (sent < frame.size()) {
        const auto written = ::send(sockfd, frame.data() + sent, frame.size() - sent, 0);

        if (written < 0) {
            if (errno == EINTR)
                continue;
            throw MB::ModbusException(MB::utils::ProtocolError);
        }

        if (written == 0)
            throw MB::ModbusException(MB::utils::ConnectionClosed);

        sent += static_cast<std::size_t>(written);
    }
}

} // namespace

Connection::Connection(const int sockfd) noexcept {
    _sockfd    = sockfd;
    _messageID = 0;
}

Connection::~Connection() {
    if (_sockfd == -1)
        return;

    ::close(_sockfd);
    _sockfd = -1;
}

// Modified 2026 by Mechatronics Academy: wait for the socket to become readable, then
// read exactly one whole MBAP frame. The previous code did a single ::recv() into a
// 1024-byte buffer and assumed a complete frame had arrived, with the result unchecked
// for short reads. Shared by awaitRequest / awaitResponse / awaitRawMessage so all three
// agree on framing.
std::vector<uint8_t> Connection::readFrame(const int timeoutMs) {
    pollfd pfd;
    pfd.fd      = _sockfd;
    pfd.events  = POLLIN;
    pfd.revents = 0;

    if (::poll(&pfd, 1, timeoutMs) <= 0)
        throw MB::ModbusException(MB::utils::Timeout);

    std::vector<uint8_t> header = recvExactly(6);
    const uint16_t length       = readBigEndian16(&header[4]);

    if (length == 0)
        throw MB::ModbusException(MB::utils::ProtocolError);

    std::vector<uint8_t> pdu = recvExactly(length);

    header.insert(header.end(), pdu.begin(), pdu.end());

    return header;
}

std::vector<uint8_t> Connection::recvExactly(const std::size_t count) {
    std::vector<uint8_t> buffer(count);
    std::size_t received = 0;

    while (received < count) {
        const auto got = ::recv(_sockfd, buffer.data() + received, count - received, 0);

        if (got < 0) {
            if (errno == EINTR)
                continue;
            throw MB::ModbusException(MB::utils::ProtocolError);
        }

        if (got == 0)
            throw MB::ModbusException(MB::utils::ConnectionClosed);

        received += static_cast<std::size_t>(got);
    }

    return buffer;
}

std::vector<uint8_t> Connection::sendRequest(const MB::ModbusRequest &req) {
    // Modified 2026 by Mechatronics Academy: advance the transaction id per request. It
    // was never incremented, so every frame on the wire carried id 0 and awaitResponse()'s
    // id check was degenerate: after one timeout the next call would accept the PREVIOUS
    // transaction's buffered payload and return a stale reading with no error.
    ++_messageID;

    const std::vector<uint8_t> frame = buildMbapFrame(_messageID, req.toRaw());

    sendAll(_sockfd, frame);

    return frame;
}

std::vector<uint8_t> Connection::sendResponse(const MB::ModbusResponse &res) {
    // Server side: echo the transaction id that awaitRequest() adopted from the client.
    const std::vector<uint8_t> frame = buildMbapFrame(_messageID, res.toRaw());

    sendAll(_sockfd, frame);

    return frame;
}

std::vector<uint8_t> Connection::sendException(const MB::ModbusException &ex) {
    const std::vector<uint8_t> frame = buildMbapFrame(_messageID, ex.toRaw());

    sendAll(_sockfd, frame);

    return frame;
}

MB::ModbusRequest Connection::awaitRequest() {
    std::vector<uint8_t> r = readFrame(_requestTimeout);

    _messageID = readBigEndian16(&r[0]);

    r.erase(r.begin(), r.begin() + 6);

    return MB::ModbusRequest::fromRaw(r);
}

MB::ModbusResponse Connection::awaitResponse() {
    std::vector<uint8_t> r = readFrame(_timeout);

    if (readBigEndian16(&r[0]) != _messageID)
        throw MB::ModbusException(MB::utils::InvalidMessageID);

    r.erase(r.begin(), r.begin() + 6);

    if (MB::ModbusException::exist(r))
        throw MB::ModbusException(r);

    return MB::ModbusResponse::fromRaw(r);
}

std::vector<uint8_t> Connection::awaitRawMessage() {
    return readFrame(_requestTimeout);
}

Connection::Connection(Connection &&moved) noexcept {
    if (_sockfd != -1 && moved._sockfd != _sockfd)
        ::close(_sockfd);

    _sockfd       = moved._sockfd;
    _messageID      = moved._messageID;
    _timeout        = moved._timeout;
    _requestTimeout = moved._requestTimeout;
    moved._sockfd   = -1;
}

Connection Connection::with(std::string addr, int port) {
    auto sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1)
        throw std::runtime_error("Cannot open socket, errno = " + std::to_string(errno));

    sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port   = ::htons(port);
    server.sin_addr   = {inet_addr(addr.c_str())};

    if (::connect(sock, reinterpret_cast<struct sockaddr *>(&server), sizeof(server)) < 0) {
        // Modified 2026 by Mechatronics Academy: close the socket before throwing.
        // Upstream fixed this in 397a169 ("Fix file descriptor leak on connection
        // failure"); fork commit c5d53cb flattened src/TCP/ -> src/ and dropped the fix
        // in the process. With the rover's shipped URDF (retry forever, 1 s delay) an
        // unreachable relay board leaked one fd per second inside the controller_manager
        // process until EMFILE.
        const int connectErrno = errno;
        ::close(sock);
        throw std::runtime_error("Cannot connect, errno = " + std::to_string(connectErrno));
    }

    return Connection(sock);
}
