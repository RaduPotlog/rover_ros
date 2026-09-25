// Copyright 2026 Mechatronics Academy
// Licensed under: MIT License <http://opensource.org/licenses/MIT>

//
// The POSIX client and server (Modbus_Tcp), against the fixes listed under "Local changes to
// this fork" in README.md.
//
// Framing, transaction ids and the error paths run MB::TCP::Connection over an AF_UNIX
// socketpair(), with the far end written and read by hand - no TCP port at all. The connect,
// bind and accept failures need real sockets: they run on loopback with kernel-assigned ports
// only, so they cannot collide with rover_modbus_driver's pid-derived e2e ports. Leaks are
// counted in /proc/self/fd.
//
// MB::TCP::Server::port() returns the constructor argument, so for Server(0) it is 0, not the
// port the kernel picked. boundPort() reads the real one back with getsockname().

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <MB/connection.hpp>
#include <MB/modbusCell.hpp>
#include <MB/modbusException.hpp>
#include <MB/modbusRequest.hpp>
#include <MB/modbusResponse.hpp>
#include <MB/modbusUtils.hpp>
#include <MB/server.hpp>

namespace
{

// Server-side waits in these tests: the frame is always already on its way, so this only
// bounds how long a failing test takes (the library default is 60 s).
constexpr int RequestWaitMs = 2000;

// Owns a raw fd until release()d, e.g. to an MB::TCP::Connection, which then closes it.
class UniqueFd
{

public:

    explicit UniqueFd(const int fd = -1) noexcept : fd_(fd) {}

    UniqueFd(UniqueFd && other) noexcept : fd_(other.release()) {}

    UniqueFd(const UniqueFd &)             = delete;
    UniqueFd & operator=(const UniqueFd &) = delete;
    UniqueFd & operator=(UniqueFd &&)      = delete;

    ~UniqueFd() { reset(); }

    [[nodiscard]] int get() const noexcept { return fd_; }

    [[nodiscard]] int release() noexcept
    {
        const int fd = fd_;
        fd_          = -1;
        return fd;
    }

    void reset() noexcept
    {
        if (fd_ >= 0) {
            ::close(fd_);
        }
        fd_ = -1;
    }

private:

    int fd_;
};

// A connected AF_UNIX stream pair: `local` goes to the MB::TCP::Connection under test, `peer`
// is the far end, driven by hand.
struct SocketPair
{
    UniqueFd local;
    UniqueFd peer;
};

SocketPair makeSocketPair()
{
    int fds[2] = {-1, -1};

    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) {
        throw std::runtime_error("socketpair() failed, errno = " + std::to_string(errno));
    }

    SocketPair pair{UniqueFd(fds[0]), UniqueFd(fds[1])};

    // Bound every blocking read on the peer, so a byte that never comes fails the test
    // instead of hanging it.
    timeval timeout{};
    timeout.tv_sec = 2;
    ::setsockopt(pair.peer.get(), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    return pair;
}

// Every fd this process has open. The iterator's own fd is counted each time, so two counts
// compare equal when nothing leaked in between.
std::size_t openFdCount()
{
    return static_cast<std::size_t>(
        std::distance(std::filesystem::directory_iterator("/proc/self/fd"),
                      std::filesystem::directory_iterator{}));
}

// The port an already-bound socket actually got, e.g. after binding port 0.
int boundPort(const int fd)
{
    sockaddr_in addr{};
    socklen_t addr_len = sizeof(addr);

    if (::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
        throw std::runtime_error("getsockname() failed, errno = " + std::to_string(errno));
    }

    return ntohs(addr.sin_port);
}

// Binds an ephemeral loopback port and closes it again: connecting to it right after fails
// fast with ECONNREFUSED.
int reserveClosedLoopbackPort()
{
    const UniqueFd probe(::socket(AF_INET, SOCK_STREAM, 0));

    if (probe.get() < 0) {
        throw std::runtime_error("Failed to create probe socket.");
    }

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = 0;

    if (::bind(probe.get(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        throw std::runtime_error("Failed to bind probe socket.");
    }

    return boundPort(probe.get());
}

// An MBAP frame built independently of the library: transaction id, protocol id 0, then the
// big-endian length of the PDU that follows.
std::vector<uint8_t> mbap(const uint16_t tid, const std::vector<uint8_t> & pdu)
{
    std::vector<uint8_t> frame{
        static_cast<uint8_t>(tid >> 8),
        static_cast<uint8_t>(tid & 0xFFu),
        0x00,
        0x00,
        static_cast<uint8_t>(pdu.size() >> 8),
        static_cast<uint8_t>(pdu.size() & 0xFFu)};

    frame.insert(frame.end(), pdu.begin(), pdu.end());

    return frame;
}

bool writeAll(const int fd, const std::vector<uint8_t> & bytes)
{
    std::size_t sent = 0;

    while (sent < bytes.size()) {
        const auto written = ::send(fd, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);

        if (written <= 0) {
            return false;
        }

        sent += static_cast<std::size_t>(written);
    }

    return true;
}

// Up to `count` bytes; fewer only if the peer closed or the receive timeout ran out.
std::vector<uint8_t> readExactly(const int fd, const std::size_t count)
{
    std::vector<uint8_t> buffer(count);

    const auto got = ::recv(fd, buffer.data(), count, MSG_WAITALL);

    buffer.resize(got < 0 ? 0 : static_cast<std::size_t>(got));

    return buffer;
}

bool hasPendingBytes(const int fd)
{
    uint8_t byte = 0;

    return ::recv(fd, &byte, 1, MSG_DONTWAIT | MSG_PEEK) > 0;
}

// sendAll() in connection.cpp sends without MSG_NOSIGNAL, so writing to a closed peer raises
// SIGPIPE. Ignored for the guard's lifetime, then the previous action is restored.
class SigpipeIgnored
{

public:

    SigpipeIgnored()
    {
        struct sigaction ignore{};
        ignore.sa_handler = SIG_IGN;
        ::sigemptyset(&ignore.sa_mask);
        ::sigaction(SIGPIPE, &ignore, &previous_);
    }

    ~SigpipeIgnored() { ::sigaction(SIGPIPE, &previous_, nullptr); }

    SigpipeIgnored(const SigpipeIgnored &)             = delete;
    SigpipeIgnored & operator=(const SigpipeIgnored &) = delete;

private:

    struct sigaction previous_{};
};

// Joins on scope exit, so a throwing await cannot leave a joinable std::thread behind.
class JoiningThread
{

public:

    template <typename Function>
    explicit JoiningThread(Function && function) : thread_(std::forward<Function>(function))
    {
    }

    ~JoiningThread()
    {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    JoiningThread(const JoiningThread &)             = delete;
    JoiningThread & operator=(const JoiningThread &) = delete;

private:

    std::thread thread_;
};

// The error code of the MB::ModbusException `call` throws, or nullopt if it throws none.
template <typename Call>
std::optional<MB::utils::MBErrorCode> thrownErrorCode(Call && call)
{
    try {
        call();
    } catch (const MB::ModbusException & ex) {
        return ex.getErrorCode();
    }

    return std::nullopt;
}

MB::ModbusRequest readContact3()
{
    return MB::ModbusRequest(255, MB::utils::ReadDiscreteInputContacts, 3, 1);
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Framing, transaction ids and error paths - socketpair() only.
// ---------------------------------------------------------------------------------------------

TEST(ConnectionFramingTest, TransactionIdAdvancesOnEveryRequest)
{
    auto sockets = makeSocketPair();
    MB::TCP::Connection client(sockets.local.release());

    const auto first  = client.sendRequest(readContact3());
    const auto second = client.sendRequest(readContact3());

    EXPECT_EQ(first, (std::vector<uint8_t>{
                         0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0xFF, 0x02, 0x00, 0x03, 0x00, 0x01}));
    ASSERT_EQ(second.size(), 12U);
    EXPECT_EQ(second[0], 0x00);
    EXPECT_EQ(second[1], 0x02);
    EXPECT_EQ(client.getMessageId(), 2);

    std::vector<uint8_t> both = first;
    both.insert(both.end(), second.begin(), second.end());

    EXPECT_EQ(readExactly(sockets.peer.get(), 24U), both);
    EXPECT_FALSE(hasPendingBytes(sockets.peer.get()));
}

TEST(ConnectionFramingTest, MbapLengthIsBigEndianAboveTwoHundredFiftyFiveBytes)
{
    auto sockets = makeSocketPair();
    MB::TCP::Connection client(sockets.local.release());
    MB::TCP::Connection server(sockets.peer.release());
    server.setRequestTimeout(RequestWaitMs);

    const std::vector<MB::ModbusCell> values(125, MB::ModbusCell::initReg(7));

    const auto frame = client.sendRequest(MB::ModbusRequest(
        255, MB::utils::WriteMultipleAnalogOutputHoldingRegisters, 0, 125, values));

    // 6 bytes of MBAP, then unit id, function, address (2), count (2), byte count and 125 x 2
    // bytes of values: a length of 257 = 0x0101.
    ASSERT_EQ(frame.size(), 263U);
    EXPECT_EQ(frame[4], 0x01);
    EXPECT_EQ(frame[5], 0x01);

    const auto request = server.awaitRequest();

    EXPECT_EQ(request.numberOfRegisters(), 125);
    ASSERT_EQ(request.registerValues().size(), 125U);
    EXPECT_EQ(request.registerValues()[124].reg(), 7);
    EXPECT_EQ(server.getMessageId(), 1);
}

TEST(ConnectionFramingTest, ARequestSplitAcrossTwoWritesIsReadAsOneFrame)
{
    auto sockets = makeSocketPair();
    MB::TCP::Connection server(sockets.local.release());
    server.setRequestTimeout(RequestWaitMs);

    const auto frame = mbap(1, {0xFF, 0x02, 0x00, 0x03, 0x00, 0x01});
    const int peer   = sockets.peer.get();

    // The sleep only splits the frame into two segments; nothing waits on it.
    const JoiningThread writer([&frame, peer] {
        writeAll(peer, std::vector<uint8_t>(frame.begin(), frame.begin() + 4));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        writeAll(peer, std::vector<uint8_t>(frame.begin() + 4, frame.end()));
    });

    const auto request = server.awaitRequest();

    EXPECT_EQ(request.registerAddress(), 3);
    EXPECT_EQ(request.numberOfRegisters(), 1);
}

TEST(ConnectionFramingTest, TwoFramesInOneWriteAreReadOneAtATime)
{
    auto sockets = makeSocketPair();
    MB::TCP::Connection server(sockets.local.release());
    server.setRequestTimeout(RequestWaitMs);

    auto bytes        = mbap(7, {0xFF, 0x02, 0x00, 0x03, 0x00, 0x01});
    const auto second = mbap(8, {0xFF, 0x01, 0x00, 0x05, 0x00, 0x02});
    bytes.insert(bytes.end(), second.begin(), second.end());

    ASSERT_TRUE(writeAll(sockets.peer.get(), bytes));

    const auto contactRead = server.awaitRequest();

    EXPECT_EQ(contactRead.functionCode(), MB::utils::ReadDiscreteInputContacts);
    EXPECT_EQ(contactRead.registerAddress(), 3);
    EXPECT_EQ(server.getMessageId(), 7);

    const auto coilRead = server.awaitRequest();

    EXPECT_EQ(coilRead.functionCode(), MB::utils::ReadDiscreteOutputCoils);
    EXPECT_EQ(coilRead.registerAddress(), 5);
    EXPECT_EQ(coilRead.numberOfRegisters(), 2);
    EXPECT_EQ(server.getMessageId(), 8);
}

// The README's stale-payload scenario: the reply to a timed-out request arrives while the next
// request is outstanding. It must be refused, not returned as the new reading.
TEST(ConnectionFramingTest, AStaleReplyAfterATimeoutIsRejectedNotReturned)
{
    auto sockets = makeSocketPair();
    MB::TCP::Connection client(sockets.local.release());
    client.setTimeout(50);

    client.sendRequest(readContact3());  // transaction 1

    EXPECT_EQ(thrownErrorCode([&] { (void)client.awaitResponse(); }), MB::utils::Timeout);

    client.sendRequest(readContact3());  // transaction 2

    ASSERT_TRUE(writeAll(sockets.peer.get(), mbap(1, {0xFF, 0x02, 0x01, 0x01})));
    ASSERT_TRUE(writeAll(sockets.peer.get(), mbap(2, {0xFF, 0x02, 0x01, 0x00})));

    EXPECT_EQ(thrownErrorCode([&] { (void)client.awaitResponse(); }),
              MB::utils::InvalidMessageID);

    const auto response = client.awaitResponse();

    ASSERT_EQ(response.registerValues().size(), 8U);
    EXPECT_FALSE(response.registerValues()[0].coil());
}

TEST(ConnectionFramingTest, AZeroLengthHeaderIsAProtocolError)
{
    auto sockets = makeSocketPair();
    MB::TCP::Connection client(sockets.local.release());

    ASSERT_TRUE(writeAll(sockets.peer.get(), {0x00, 0x01, 0x00, 0x00, 0x00, 0x00}));

    EXPECT_EQ(thrownErrorCode([&] { (void)client.awaitResponse(); }), MB::utils::ProtocolError);
}

TEST(ConnectionFramingTest, APeerClosingMidFrameIsConnectionClosed)
{
    auto sockets = makeSocketPair();
    MB::TCP::Connection client(sockets.local.release());

    ASSERT_TRUE(writeAll(sockets.peer.get(), {0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0xFF}));
    sockets.peer.reset();

    EXPECT_EQ(thrownErrorCode([&] { (void)client.awaitResponse(); }),
              MB::utils::ConnectionClosed);
}

TEST(ConnectionFramingTest, AnExceptionReplyIsRaisedAsTheDevicesError)
{
    auto sockets = makeSocketPair();
    MB::TCP::Connection client(sockets.local.release());

    client.sendRequest(readContact3());

    ASSERT_TRUE(writeAll(sockets.peer.get(), mbap(1, {0xFF, 0x82, 0x02})));

    try {
        (void)client.awaitResponse();
        FAIL() << "the exception reply was returned as a response";
    } catch (const MB::ModbusException & ex) {
        EXPECT_EQ(ex.getErrorCode(), MB::utils::IllegalDataAddress);
        EXPECT_EQ(ex.functionCode(), MB::utils::ReadDiscreteInputContacts);
        EXPECT_EQ(ex.slaveID(), 255);
    }
}

TEST(ConnectionFramingTest, ASendToAClosedPeerThrows)
{
    const SigpipeIgnored sigpipeIgnored;

    auto sockets = makeSocketPair();
    MB::TCP::Connection client(sockets.local.release());
    sockets.peer.reset();

    EXPECT_EQ(thrownErrorCode([&] { client.sendRequest(readContact3()); }),
              MB::utils::ProtocolError);
}

TEST(ConnectionFramingTest, SetTimeoutBoundsTheResponseWait)
{
    auto sockets = makeSocketPair();
    MB::TCP::Connection client(sockets.local.release());
    client.setTimeout(20);

    const auto start = std::chrono::steady_clock::now();

    EXPECT_EQ(thrownErrorCode([&] { (void)client.awaitResponse(); }), MB::utils::Timeout);

    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - start)
                               .count();

    // Well under the 500 ms default, so the configured 20 ms is what applied.
    EXPECT_LT(elapsedMs, 400);
}

// ---------------------------------------------------------------------------------------------
// Connection lifetime.
// ---------------------------------------------------------------------------------------------

TEST(ConnectionLifetimeTest, TimeoutsMessageIdAndSocketSurviveMoves)
{
    auto sockets = makeSocketPair();
    const int fd = sockets.local.release();

    MB::TCP::Connection original(fd);
    original.setTimeout(123);
    original.setRequestTimeout(4567);
    original.setMessageId(9);

    MB::TCP::Connection moveConstructed(std::move(original));

    EXPECT_EQ(moveConstructed.getSockfd(), fd);
    EXPECT_EQ(moveConstructed.getTimeout(), 123);
    EXPECT_EQ(moveConstructed.getRequestTimeout(), 4567);
    EXPECT_EQ(moveConstructed.getMessageId(), 9);
    EXPECT_EQ(original.getSockfd(), -1);

    MB::TCP::Connection moveAssigned;
    moveAssigned = std::move(moveConstructed);

    EXPECT_EQ(moveAssigned.getSockfd(), fd);
    EXPECT_EQ(moveAssigned.getTimeout(), 123);
    EXPECT_EQ(moveAssigned.getRequestTimeout(), 4567);
    EXPECT_EQ(moveAssigned.getMessageId(), 9);
    EXPECT_EQ(moveConstructed.getSockfd(), -1);
}

TEST(ConnectionLifetimeTest, DestructionClosesTheSocket)
{
    auto sockets = makeSocketPair();

    {
        const MB::TCP::Connection connection(sockets.local.release());
    }

    uint8_t byte = 0;

    EXPECT_EQ(::recv(sockets.peer.get(), &byte, 1, 0), 0);
}

// Connection::with() used to leak the socket whenever connect() failed; with the shipped URDF
// retrying once a second, that ran the controller_manager out of fds.
TEST(ConnectionLifetimeTest, AFailedConnectDoesNotLeakTheSocket)
{
    const int closedPort = reserveClosedLoopbackPort();
    const auto before    = openFdCount();

    for (int i = 0; i < 20; ++i) {
        EXPECT_THROW((void)MB::TCP::Connection::with("127.0.0.1", closedPort), std::runtime_error);
    }

    EXPECT_EQ(openFdCount(), before);

    try {
        (void)MB::TCP::Connection::with("127.0.0.1", closedPort);
        FAIL() << "connected to a closed port";
    } catch (const std::runtime_error & ex) {
        const std::string expected = "Cannot connect, errno = " + std::to_string(ECONNREFUSED);
        EXPECT_NE(std::string(ex.what()).find(expected), std::string::npos) << ex.what();
    }
}

// ---------------------------------------------------------------------------------------------
// Server - loopback, kernel-assigned ports only.
// ---------------------------------------------------------------------------------------------

// accept() failing used to reach a bare `throw;` and std::terminate(): a regression aborts
// this test binary rather than failing one case.
TEST(ServerTest, AnAcceptFailureReturnsNulloptInsteadOfTerminating)
{
    MB::TCP::Server server(0);
    const int fd = server.nativeHandle();

    // Non-blocking with no client waiting: accept() fails straight away with EAGAIN.
    ASSERT_EQ(::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK), 0);

    EXPECT_FALSE(server.awaitConnection().has_value());
}

TEST(ServerTest, AFailedBindDoesNotLeakTheSocket)
{
    // Listening without SO_REUSEADDR / SO_REUSEPORT, so the Server's own bind() to the same
    // port must fail.
    const UniqueFd holder(::socket(AF_INET, SOCK_STREAM, 0));
    ASSERT_GE(holder.get(), 0);

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = 0;

    ASSERT_EQ(::bind(holder.get(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
    ASSERT_EQ(::listen(holder.get(), 1), 0);

    const int port    = boundPort(holder.get());
    const auto before = openFdCount();

    for (int i = 0; i < 20; ++i) {
        EXPECT_THROW({ MB::TCP::Server server(port); }, std::runtime_error);
    }

    EXPECT_EQ(openFdCount(), before);

    try {
        MB::TCP::Server server(port);
        FAIL() << "bound a port that is already listening";
    } catch (const std::runtime_error & ex) {
        EXPECT_NE(std::string(ex.what()).find("Cannot bind socket"), std::string::npos)
            << ex.what();
    }
}

TEST(ServerTest, ConstructionAndDestructionLeaveNoSocketBehind)
{
    const auto before = openFdCount();

    for (int i = 0; i < 20; ++i) {
        MB::TCP::Server server(0);
    }

    EXPECT_EQ(openFdCount(), before);
}

// Single-threaded: connect() completes against the listen backlog before accept() runs.
TEST(ServerTest, AClientAndServerCompleteATransactionOverLoopback)
{
    MB::TCP::Server server(0);
    const int port = boundPort(server.nativeHandle());

    auto client   = MB::TCP::Connection::with("127.0.0.1", port);
    auto accepted = server.awaitConnection();

    ASSERT_TRUE(accepted.has_value());
    accepted->setRequestTimeout(RequestWaitMs);

    client.sendRequest(MB::ModbusRequest(255, MB::utils::ReadDiscreteOutputCoils, 8, 12));

    const auto request = accepted->awaitRequest();

    EXPECT_EQ(request.slaveID(), 255);
    EXPECT_EQ(request.functionCode(), MB::utils::ReadDiscreteOutputCoils);
    EXPECT_EQ(request.registerAddress(), 8);
    EXPECT_EQ(request.numberOfRegisters(), 12);
    EXPECT_EQ(accepted->getMessageId(), 1);

    const std::vector<bool> bits = {
        true, false, false, true, false, true, true, false, false, false, true, true};

    std::vector<MB::ModbusCell> cells;
    for (const bool bit : bits) {
        cells.push_back(MB::ModbusCell::initCoil(bit));
    }

    accepted->sendResponse(MB::ModbusResponse(request.slaveID(), request.functionCode(),
                                              request.registerAddress(),
                                              request.numberOfRegisters(), cells));

    const auto response = client.awaitResponse();
    const auto & values = response.registerValues();

    // 12 coils travel as two whole bytes, so 16 cells come back; the padding is false.
    ASSERT_EQ(values.size(), 16U);
    for (std::size_t i = 0; i < bits.size(); ++i) {
        EXPECT_EQ(values[i].coil(), bits[i]) << "coil " << i;
    }
}
