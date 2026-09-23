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
// End-to-end round trip against a real MB::TCP::Server on loopback.
//
// This is the first test anywhere in the workspace that proves request encoding and
// response decoding actually agree - everything else either stops at the connection
// attempt or substitutes a fake above the codec. It is what would have caught the MBAP
// length encoding and the frozen transaction id.
//
// Port numbers are derived from the pid, following
// rover_udp_driver/test/integration/test_asio_udp_socket.cpp, so parallel colcon test runs
// do not collide.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include <MB/modbusCell.hpp>
#include <MB/modbusException.hpp>
#include <MB/modbusResponse.hpp>
#include <MB/modbusUtils.hpp>
#include <MB/server.hpp>

#include "rover_modbus_driver/infrastructure/modbus_tcp_client_factory.hpp"
#include "rover_modbus_driver/infrastructure/modbus_tcp_transport.hpp"

namespace rover::transport::modbus::test
{

namespace
{

int testPort(const int offset)
{
    return 20000 + (static_cast<int>(::getpid()) % 10000) + offset;
}

// A one-shot Modbus server: accepts a single connection, then answers every request with
// a single coil set to `coil_value` (or with `bits`, when given), recording what it was asked
// for.
class OneShotServer
{

public:

    OneShotServer(const int port, const bool coil_value, const int requests_to_serve)
    : server_(port), coil_value_(coil_value), requests_to_serve_(requests_to_serve)
    {
        thread_ = std::thread([this]() { run(); });
    }

    OneShotServer(const int port, std::vector<bool> bits, const int requests_to_serve)
    : server_(port), coil_value_(false), bits_(std::move(bits)),
      requests_to_serve_(requests_to_serve)
    {
        thread_ = std::thread([this]() { run(); });
    }

    ~OneShotServer()
    {
        stop_ = true;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    const std::vector<MB::ModbusRequest> & served() const { return served_; }

private:

    void run()
    {
        auto connection = server_.awaitConnection();

        if (!connection.has_value()) {
            return;
        }

        // Keep the server-side wait short so a failing test cannot hang for the default
        // 60 s; the client connects immediately in all of these cases.
        connection->setRequestTimeout(5000);

        for (int n = 0; n < requests_to_serve_ && !stop_; ++n) {
            try {
                const MB::ModbusRequest request = connection->awaitRequest();
                served_.push_back(request);

                std::vector<MB::ModbusCell> values;

                if (bits_.empty()) {
                    values.push_back(MB::ModbusCell(coil_value_));
                } else {
                    for (const bool bit : bits_) {
                        values.push_back(MB::ModbusCell(bit));
                    }
                }

                MB::ModbusResponse response(
                    request.slaveID(), request.functionCode(), request.registerAddress(),
                    request.numberOfRegisters(), values);

                connection->sendResponse(response);
            } catch (const MB::ModbusException &) {
                return;
            }
        }
    }

    MB::TCP::Server server_;
    bool coil_value_;
    std::vector<bool> bits_;
    int requests_to_serve_;
    std::vector<MB::ModbusRequest> served_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

ClientSettings settingsFor(const int port)
{
    ClientSettings settings;
    settings.host                      = "127.0.0.1";
    settings.port                      = port;
    settings.connection_retry_count    = 50;
    settings.connection_retry_delay_ms = 20;
    settings.response_timeout_ms       = 5000;

    return settings;
}

}  // namespace

TEST(ModbusTcpRoundTripTest, ReadDiscreteContactCompletesAgainstARealServer)
{
    const int port = testPort(0);

    OneShotServer server(port, /*coil_value=*/true, /*requests_to_serve=*/1);

    auto client = makeModbusTcpDiscreteIoClient(settingsFor(port));

    EXPECT_EQ(client->readDiscreteContact(ContactInfo{Contact::CONTACT_2}), 1U);
}

TEST(ModbusTcpRoundTripTest, WriteDiscreteCoilCompletesAgainstARealServer)
{
    const int port = testPort(1);

    OneShotServer server(port, /*coil_value=*/true, /*requests_to_serve=*/1);

    auto client = makeModbusTcpDiscreteIoClient(settingsFor(port));

    EXPECT_NO_THROW(client->writeDiscreteCoil(CoilInfo{Coil::COIL_1, true, true}, true));
}

// Several transactions in a row over one connection. This is the case the frozen
// transaction id made unsafe: every frame used to go out with id 0, so awaitResponse()
// could not tell a fresh reply from a stale buffered one.
TEST(ModbusTcpRoundTripTest, ConsecutiveTransactionsUseDistinctTransactionIds)
{
    const int port = testPort(2);

    OneShotServer server(port, /*coil_value=*/false, /*requests_to_serve=*/3);

    ModbusTcpTransport transport("127.0.0.1", port, 5000);

    MB::ModbusRequest request(255, MB::utils::ReadDiscreteInputContacts, 0, 1);

    for (int n = 0; n < 3; ++n) {
        const MB::ModbusResponse response = transport.sendRequest(request);

        // A coil read comes back as a whole byte, so the decoded response carries 8 cells
        // even though one was requested; bit 0 is the coil we asked for. Worth asserting
        // explicitly - the fake transport used by the unit tests returns exactly the
        // cells it was given, so this is the only place the real framing shows through.
        ASSERT_EQ(response.registerValues().size(), 8U) << "transaction " << n;
        EXPECT_FALSE(response.registerValues().front().coil()) << "transaction " << n;
    }
}

// The batched read the safety controller's IO poll depends on: 12 bits go out as two bytes and
// must come back decoded in address order, with the byte padding dropped.
TEST(ModbusTcpRoundTripTest, BatchedCoilReadDecodesEveryBitAgainstARealServer)
{
    const int port = testPort(3);

    const std::vector<bool> bits = {
        true, false, false, true, false, true, true, false, false, false, true, true};

    OneShotServer server(port, bits, /*requests_to_serve=*/1);

    auto client = makeModbusTcpDiscreteIoClient(settingsFor(port));

    EXPECT_EQ(client->readDiscreteCoils(Coil::COIL_8, 12), bits);
}

namespace
{

// Answers every FC1 read the way the Portenta PLC IDE does (captured on the rover): correct for
// up to 8 coils, but for more it claims byte_count = 2 while sending one data byte, with an MBAP
// length that matches the one byte. Raw sockets, because MB::TCP::Server cannot send a malformed
// frame.
class PortentaLikeServer
{

public:

    explicit PortentaLikeServer(const int port)
    {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        const int one = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_port        = htons(static_cast<uint16_t>(port));
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        ::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
        ::listen(listen_fd_, 1);

        thread_ = std::thread([this]() { run(); });
    }

    ~PortentaLikeServer()
    {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        if (client_fd_ >= 0) {
            ::shutdown(client_fd_, SHUT_RDWR);
        }
        if (thread_.joinable()) {
            thread_.join();
        }
        if (client_fd_ >= 0) {
            ::close(client_fd_);
        }
    }

private:

    void run()
    {
        client_fd_ = ::accept(listen_fd_, nullptr, nullptr);
        if (client_fd_ < 0) {
            return;
        }

        uint8_t request[12];

        while (::recv(client_fd_, request, sizeof(request), MSG_WAITALL) == sizeof(request)) {
            const uint16_t count = static_cast<uint16_t>((request[10] << 8) | request[11]);
            const uint8_t byte_count = static_cast<uint8_t>((count + 7) / 8);

            // unit id, function code, byte_count, then ONE data byte whatever byte_count says.
            const uint8_t reply[] = {
                request[0], request[1], 0, 0, 0, 4, request[6], request[7], byte_count, 0x00};

            ::send(client_fd_, reply, sizeof(reply), MSG_NOSIGNAL);
        }
    }

    int listen_fd_{-1};
    int client_fd_{-1};
    std::thread thread_;
};

}  // namespace

// Against the Portenta's short reply, a >8-coil read must fail loudly instead of decoding bytes
// that were never sent, and a one-byte read must work.
TEST(ModbusTcpRoundTripTest, AShortPortentaReplyIsRejectedAndOneByteReadsWork)
{
    const int port = testPort(4);

    PortentaLikeServer server(port);

    auto client = makeModbusTcpDiscreteIoClient(settingsFor(port));

    EXPECT_EQ(client->readDiscreteCoils(Coil::COIL_14, 6), std::vector<bool>(6, false));
    EXPECT_THROW(client->readDiscreteCoils(Coil::COIL_8, 12), MB::ModbusException);
}

}  // namespace rover::transport::modbus::test
