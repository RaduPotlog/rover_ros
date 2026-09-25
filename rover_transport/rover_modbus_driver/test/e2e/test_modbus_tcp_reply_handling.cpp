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

//
// The client over the real TCP transport, against a scripted loopback server that records
// every request frame byte for byte and answers with whatever PDU the test scripts.
//
// Two things are pinned here. First, the golden request frames: the exact MBAP header and PDU
// each DiscreteIoPort operation puts on the wire. This link carries the safety PLC's watchdog
// heartbeat and E-Stop coils, so those bytes must not change when the code above the socket
// does. Second, the reply-handling table: which malformed or unexpected replies the client
// rejects while keeping the connection, and which drop it. Unlike the unit tests' fake, the
// replies here go through the real frame decoder.
//
// Raw sockets, because MB::TCP::Server cannot send a malformed or mismatched reply. The port
// is kernel-assigned (bind to 0, read back with getsockname()), so it cannot collide with
// test_modbus_tcp_roundtrip's pid-derived ports.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <MB/modbusException.hpp>
#include <MB/modbusUtils.hpp>

#include "fakes/fake_logger.hpp"
#include "rover_modbus_driver/application/modbus_discrete_io_client.hpp"
#include "rover_modbus_driver/infrastructure/modbus_tcp_transport.hpp"

namespace rover::transport::modbus::test
{

namespace
{

using Frame = std::vector<uint8_t>;

// Accepts connections one after another on 127.0.0.1, reads 12-byte request frames (every
// FC1/FC2/FC5 request this client sends is exactly that long), records each one, and replies
// with the request's transaction id, protocol 0, the right MBAP length, and the PDU the
// responder returns.
class ScriptedServer
{

public:

    // Returns the reply PDU (unit id, function code, data) for the request_index-th request
    // this server has seen, counted across connections.
    using Responder = std::function<Frame(const Frame & request_frame, int request_index)>;

    explicit ScriptedServer(Responder responder)
    : responder_(std::move(responder))
    {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) {
            throw std::runtime_error("socket() failed");
        }

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_port        = 0;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
            ::listen(listen_fd_, 1) != 0)
        {
            ::close(listen_fd_);
            throw std::runtime_error("bind()/listen() failed");
        }

        socklen_t addr_len = sizeof(addr);
        if (::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
            ::close(listen_fd_);
            throw std::runtime_error("getsockname() failed");
        }
        port_ = ntohs(addr.sin_port);

        thread_ = std::thread([this]() { run(); });
    }

    ~ScriptedServer()
    {
        stopping_ = true;

        ::shutdown(listen_fd_, SHUT_RDWR);

        const int client_fd = client_fd_.load();
        if (client_fd >= 0) {
            ::shutdown(client_fd, SHUT_RDWR);
        }

        if (thread_.joinable()) {
            thread_.join();
        }

        ::close(listen_fd_);
    }

    ScriptedServer(const ScriptedServer &)             = delete;
    ScriptedServer & operator=(const ScriptedServer &) = delete;

    int port() const { return port_; }

    int connectionsAccepted() const { return connections_accepted_.load(); }

    std::vector<Frame> frames() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return frames_;
    }

private:

    void run()
    {
        int request_index = 0;

        while (!stopping_) {
            const int fd = ::accept(listen_fd_, nullptr, nullptr);
            if (fd < 0) {
                return;
            }

            // Published before stopping_ is re-checked, so the destructor either sees this fd
            // and shuts it down, or this thread sees stopping_ and leaves on its own.
            client_fd_ = fd;
            connections_accepted_++;

            if (stopping_) {
                client_fd_ = -1;
                ::close(fd);
                return;
            }

            Frame request(12);

            while (::recv(fd, request.data(), request.size(), MSG_WAITALL) ==
                static_cast<ssize_t>(request.size()))
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    frames_.push_back(request);
                }

                const Frame pdu = responder_(request, request_index++);

                Frame reply = {
                    request[0], request[1], 0x00, 0x00,
                    static_cast<uint8_t>((pdu.size() >> 8) & 0xFFU),
                    static_cast<uint8_t>(pdu.size() & 0xFFU)};
                reply.insert(reply.end(), pdu.begin(), pdu.end());

                ::send(fd, reply.data(), reply.size(), MSG_NOSIGNAL);
            }

            client_fd_ = -1;
            ::close(fd);
        }
    }

    Responder responder_;
    int listen_fd_{-1};
    int port_{0};
    std::atomic<int> client_fd_{-1};
    std::atomic<bool> stopping_{false};
    std::atomic<int> connections_accepted_{0};
    mutable std::mutex mutex_;
    std::vector<Frame> frames_;
    std::thread thread_;
};

// A read answered with one data byte whose bit 0 is set: that coil/contact reads 1, the other
// seven cells are padding. A write single coil is answered with its own echo, as a device does.
Frame answerEveryReadWithOneSetBit(const Frame & request, int)
{
    const uint8_t function = request[7];

    if (function == MB::utils::WriteSingleDiscreteOutputCoil) {
        return Frame(request.begin() + 6, request.end());
    }

    return {0xFF, function, 0x01, 0x01};
}

// The client exactly as the factory builds it, except that the logger is a fake the test can
// read back.
std::unique_ptr<ModbusDiscreteIoClient> makeClient(
    const int port, const std::shared_ptr<FakeLogger> & logger)
{
    ClientSettings settings;
    settings.host                      = "127.0.0.1";
    settings.port                      = port;
    settings.connection_retry_count    = 50;
    settings.connection_retry_delay_ms = 20;

    return std::make_unique<ModbusDiscreteIoClient>(
        [port]() -> std::unique_ptr<ModbusTransportPort> {
            return std::make_unique<ModbusTcpTransport>("127.0.0.1", port, 2000);
        },
        settings, logger);
}

template <typename Op>
std::optional<MB::ModbusException> modbusExceptionFrom(Op op)
{
    try {
        op();
    } catch (const MB::ModbusException & e) {
        return e;
    }

    return std::nullopt;
}

}  // namespace

// The golden frames. Transaction ids count up from 1 on the one connection; protocol id 0;
// length 6 (unit id + 5 PDU bytes); unit id 255; FC2 for contacts, FC1 for coils, FC5 for
// writes; the Contact/Coil enum value is the address; FC5 drives FF 00 / 00 00.
TEST(ModbusTcpReplyHandlingTest, EveryOperationPutsTheSameBytesOnTheWire)
{
    ScriptedServer server(answerEveryReadWithOneSetBit);
    auto logger = std::make_shared<FakeLogger>();
    auto client = makeClient(server.port(), logger);

    EXPECT_EQ(client->readDiscreteContact(ContactInfo{Contact::CONTACT_3}), 1U);
    EXPECT_EQ(client->readDiscreteCoil(CoilInfo{Coil::COIL_5, false, false}), 1U);
    client->writeDiscreteCoil(CoilInfo{Coil::COIL_4, false, true}, true);
    client->writeDiscreteCoil(CoilInfo{Coil::COIL_4, false, true}, false);
    EXPECT_EQ(
        client->readDiscreteCoils(Coil::COIL_14, 6),
        (std::vector<bool>{true, false, false, false, false, false}));
    EXPECT_EQ(
        client->readDiscreteContacts(Contact::CONTACT_0, 2), (std::vector<bool>{true, false}));

    const std::vector<Frame> expected = {
        {0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0xFF, 0x02, 0x00, 0x03, 0x00, 0x01},
        {0x00, 0x02, 0x00, 0x00, 0x00, 0x06, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x01},
        {0x00, 0x03, 0x00, 0x00, 0x00, 0x06, 0xFF, 0x05, 0x00, 0x04, 0xFF, 0x00},
        {0x00, 0x04, 0x00, 0x00, 0x00, 0x06, 0xFF, 0x05, 0x00, 0x04, 0x00, 0x00},
        {0x00, 0x05, 0x00, 0x00, 0x00, 0x06, 0xFF, 0x01, 0x00, 0x0E, 0x00, 0x06},
        {0x00, 0x06, 0x00, 0x00, 0x00, 0x06, 0xFF, 0x02, 0x00, 0x00, 0x00, 0x02},
    };

    EXPECT_EQ(server.frames(), expected);
    EXPECT_EQ(server.connectionsAccepted(), 1);
    EXPECT_TRUE(logger->errors.empty());
    EXPECT_TRUE(client->isConnected());
}

// A register reply to a contact read decodes, but its cell is not a coil. The client returns
// the sentinel and keeps the connection.
TEST(ModbusTcpReplyHandlingTest, ARegisterReplyToAContactReadYieldsTheUnavailableSentinel)
{
    ScriptedServer server([](const Frame &, int) -> Frame {
        return {0xFF, 0x03, 0x02, 0x00, 0x01};
    });
    auto logger = std::make_shared<FakeLogger>();
    auto client = makeClient(server.port(), logger);

    EXPECT_EQ(
        client->readDiscreteContact(ContactInfo{Contact::CONTACT_0}), kDiscreteReadUnavailable);

    EXPECT_TRUE(client->isConnected());
    EXPECT_TRUE(logger->errors.empty());
    EXPECT_EQ(server.connectionsAccepted(), 1);
}

// A well-formed reply with byte count 0 carries no values. The client rejects it with
// NumberOfValuesInvalid but keeps the connection, and the next read on it works.
TEST(ModbusTcpReplyHandlingTest, AZeroByteCountReplyThrowsButKeepsTheConnection)
{
    ScriptedServer server([](const Frame &, const int request_index) -> Frame {
        if (request_index == 0) {
            return {0xFF, 0x02, 0x00};
        }
        return {0xFF, 0x02, 0x01, 0x01};
    });
    auto logger = std::make_shared<FakeLogger>();
    auto client = makeClient(server.port(), logger);

    const auto e = modbusExceptionFrom([&client]() {
        client->readDiscreteContact(ContactInfo{Contact::CONTACT_0});
    });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::NumberOfValuesInvalid);
    EXPECT_EQ(logger->errors, (std::vector<std::string>{"Failed to read contact"}));
    EXPECT_TRUE(client->isConnected());

    EXPECT_EQ(client->readDiscreteContact(ContactInfo{Contact::CONTACT_0}), 1U);
    EXPECT_EQ(server.connectionsAccepted(), 1);
}

// A register reply to a batched coil read has enough cells, but they are not coils. Rejected
// with the request's function code, connection kept.
TEST(ModbusTcpReplyHandlingTest, ARegisterReplyToABatchedCoilReadThrowsButKeepsTheConnection)
{
    ScriptedServer server([](const Frame &, int) -> Frame {
        return {0xFF, 0x03, 0x04, 0x00, 0x01, 0x00, 0x00};
    });
    auto logger = std::make_shared<FakeLogger>();
    auto client = makeClient(server.port(), logger);

    const auto e = modbusExceptionFrom([&client]() { client->readDiscreteCoils(Coil::COIL_0, 2); });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::NumberOfValuesInvalid);
    EXPECT_EQ(e->slaveID(), 255U);
    EXPECT_EQ(e->functionCode(), MB::utils::ReadDiscreteOutputCoils);
    EXPECT_EQ(logger->errors, (std::vector<std::string>{"Failed to read coils"}));
    EXPECT_TRUE(client->isConnected());
    EXPECT_EQ(server.connectionsAccepted(), 1);
}

// A Modbus exception PDU from the device is thrown by the transport itself, so the client
// logs it, drops the connection and rethrows.
TEST(ModbusTcpReplyHandlingTest, ADeviceExceptionReplyDropsTheConnection)
{
    ScriptedServer server([](const Frame &, int) -> Frame { return {0xFF, 0x82, 0x02}; });
    auto logger = std::make_shared<FakeLogger>();
    auto client = makeClient(server.port(), logger);

    const auto e = modbusExceptionFrom([&client]() {
        client->readDiscreteContact(ContactInfo{Contact::CONTACT_0});
    });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::IllegalDataAddress);

    ASSERT_EQ(logger->errors.size(), 2U);
    EXPECT_EQ(
        logger->errors[0].rfind("Modbus exception: Error on slave 255 - Illegal data address", 0),
        0U)
        << logger->errors[0];
    EXPECT_EQ(logger->errors[1], "Failed to read contact");
    EXPECT_FALSE(client->isConnected());
}

}  // namespace rover::transport::modbus::test
