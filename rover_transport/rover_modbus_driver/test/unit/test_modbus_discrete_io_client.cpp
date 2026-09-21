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
// Unit tests for ModbusDiscreteIoClient - what it actually puts on the wire.
//
// This is new coverage. Before the extraction, the only Modbus test exercised the
// connection-refused path, and everything above the client was driven through a fake that
// sat ABOVE the encoding - so nothing asserted the slave id, function codes or addresses
// the device actually receives. Injecting the transport factory is what makes this
// possible; these run in milliseconds with no socket and no ROS context.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>

#include <MB/modbusUtils.hpp>

#include "fakes/fake_logger.hpp"
#include "fakes/fake_modbus_transport.hpp"
#include "rover_modbus_driver/application/modbus_discrete_io_client.hpp"

namespace rover::transport::modbus::test
{

namespace
{

ClientSettings settings()
{
    ClientSettings s;
    s.host                      = "127.0.0.1";
    s.port                      = 502;
    s.connection_retry_count    = 1;
    s.connection_retry_delay_ms = 0;

    return s;
}

struct Fixture
{
    std::shared_ptr<Journal> journal{std::make_shared<Journal>()};
    std::shared_ptr<FakeLogger> logger{std::make_shared<FakeLogger>()};
    std::unique_ptr<ModbusDiscreteIoClient> client;

    Fixture()
    {
        auto journal_copy = journal;

        client = std::make_unique<ModbusDiscreteIoClient>(
            [journal_copy]() -> std::unique_ptr<ModbusTransportPort> {
                return std::make_unique<FakeModbusTransport>(journal_copy);
            },
            settings(), logger);
    }
};

}  // namespace

TEST(ModbusDiscreteIoClientTest, EmptyHostThrowsInvalidArgument)
{
    auto journal = std::make_shared<Journal>();
    auto bad     = settings();
    bad.host     = "";

    EXPECT_THROW(
        ModbusDiscreteIoClient(
            [journal]() -> std::unique_ptr<ModbusTransportPort> {
                return std::make_unique<FakeModbusTransport>(journal);
            },
            bad, std::make_shared<FakeLogger>()),
        std::invalid_argument);
}

TEST(ModbusDiscreteIoClientTest, ReadDiscreteContactEncodesFunctionCodeAndAddress)
{
    Fixture f;
    f.journal->coil_value = true;

    EXPECT_EQ(f.client->readDiscreteContact(ContactInfo{Contact::CONTACT_3}), 1U);

    ASSERT_EQ(f.journal->requests.size(), 1U);

    const auto & req = f.journal->requests.front();
    EXPECT_EQ(req.slaveID(), ModbusDiscreteIoClient::kModbusDeviceId);
    EXPECT_EQ(req.functionCode(), MB::utils::ReadDiscreteInputContacts);
    EXPECT_EQ(req.registerAddress(), 3U);
    EXPECT_EQ(req.numberOfRegisters(), 1U);
}

TEST(ModbusDiscreteIoClientTest, ReadDiscreteCoilEncodesFunctionCodeAndAddress)
{
    Fixture f;
    f.journal->coil_value = false;

    EXPECT_EQ(f.client->readDiscreteCoil(CoilInfo{Coil::COIL_5, false, false}), 0U);

    ASSERT_EQ(f.journal->requests.size(), 1U);

    const auto & req = f.journal->requests.front();
    EXPECT_EQ(req.slaveID(), ModbusDiscreteIoClient::kModbusDeviceId);
    EXPECT_EQ(req.functionCode(), MB::utils::ReadDiscreteOutputCoils);
    EXPECT_EQ(req.registerAddress(), 5U);
    EXPECT_EQ(req.numberOfRegisters(), 1U);
}

TEST(ModbusDiscreteIoClientTest, WriteDiscreteCoilEncodesFunctionCodeAddressAndValue)
{
    Fixture f;

    f.client->writeDiscreteCoil(CoilInfo{Coil::COIL_4, false, true}, true);

    ASSERT_EQ(f.journal->requests.size(), 1U);

    const auto & req = f.journal->requests.front();
    EXPECT_EQ(req.slaveID(), ModbusDiscreteIoClient::kModbusDeviceId);
    EXPECT_EQ(req.functionCode(), MB::utils::WriteSingleDiscreteOutputCoil);
    EXPECT_EQ(req.registerAddress(), 4U);
    ASSERT_EQ(req.registerValues().size(), 1U);
    EXPECT_TRUE(req.registerValues().front().coil());
}

// The safety-relevant one: a coil the board marks as read-only must never be driven.
// This guard existed but nothing covered it.
TEST(ModbusDiscreteIoClientTest, WriteToNonEngageableCoilSendsNothingAndLogsAnError)
{
    Fixture f;

    f.client->writeDiscreteCoil(CoilInfo{Coil::COIL_0, false, /*is_coil_engage_allowed=*/false}, true);

    EXPECT_TRUE(f.journal->requests.empty());
    ASSERT_EQ(f.logger->errors.size(), 1U);
    EXPECT_NE(f.logger->errors.front().find("not allowed"), std::string::npos);
}

// A device answering with no values does NOT come back as kDiscreteReadUnavailable:
// MB::ModbusResponse::registerValues() throws NumberOfValuesInvalid before the client can
// inspect the vector. Pinning that down because the sentinel reads as though it were the
// short-reply path, and it is not.
TEST(ModbusDiscreteIoClientTest, EmptyResponseSurfacesAsAModbusExceptionNotTheSentinel)
{
    Fixture f;
    f.journal->reply_empty = true;

    EXPECT_THROW(
        f.client->readDiscreteContact(ContactInfo{Contact::CONTACT_0}), MB::ModbusException);

    EXPECT_FALSE(f.logger->errors.empty());
}

TEST(ModbusDiscreteIoClientTest, TransportExceptionPropagatesAndIsLogged)
{
    Fixture f;
    f.journal->throw_error = MB::utils::Timeout;

    EXPECT_THROW(
        f.client->readDiscreteContact(ContactInfo{Contact::CONTACT_0}), MB::ModbusException);

    EXPECT_FALSE(f.logger->errors.empty());
}

TEST(ModbusDiscreteIoClientTest, GivesUpAfterConfiguredNumberOfConnectionAttempts)
{
    auto logger     = std::make_shared<FakeLogger>();
    auto conf       = settings();
    conf.connection_retry_count    = 3;
    conf.connection_retry_delay_ms = 0;

    int attempts = 0;

    EXPECT_THROW(
        ModbusDiscreteIoClient(
            [&attempts]() -> std::unique_ptr<ModbusTransportPort> {
                ++attempts;
                throw std::runtime_error("refused");
            },
            conf, logger),
        std::runtime_error);

    EXPECT_EQ(attempts, 3);
    EXPECT_EQ(logger->warnings.size(), 3U);
}

TEST(ModbusDiscreteIoClientTest, RetriesUntilTheTransportFactorySucceeds)
{
    auto journal = std::make_shared<Journal>();
    auto conf    = settings();
    conf.connection_retry_count    = 0;  // forever
    conf.connection_retry_delay_ms = 0;

    int attempts = 0;

    ModbusDiscreteIoClient client(
        [&attempts, journal]() -> std::unique_ptr<ModbusTransportPort> {
            if (++attempts < 3) {
                throw std::runtime_error("not yet");
            }
            return std::make_unique<FakeModbusTransport>(journal);
        },
        conf, std::make_shared<FakeLogger>());

    EXPECT_EQ(attempts, 3);
}

// The client owns its transport; destroying it must close the socket. Before the move the
// port had no virtual destructor, so ~ModbusTcpConnection never ran at all.
TEST(ModbusDiscreteIoClientTest, DestructorClosesTheTransport)
{
    auto journal = std::make_shared<Journal>();

    {
        ModbusDiscreteIoClient client(
            [journal]() -> std::unique_ptr<ModbusTransportPort> {
                return std::make_unique<FakeModbusTransport>(journal);
            },
            settings(), std::make_shared<FakeLogger>());

        EXPECT_FALSE(journal->closed);
    }

    EXPECT_TRUE(journal->closed);
}


// --- Reconnection --------------------------------------------------------------------------
//
// The transport used to be built once in the constructor and never rebuilt, so any drop after
// on_configure() was permanent until the whole ros2_control component was re-configured. On the
// safety path that meant one Modbus hiccup disabled the link for good.

namespace
{

// Builds a client whose factory counts its calls and can be made to fail, so a reconnection is
// observable without a socket.
struct ReconnectFixture
{
    std::shared_ptr<Journal> journal{std::make_shared<Journal>()};
    std::shared_ptr<FakeLogger> logger{std::make_shared<FakeLogger>()};
    std::unique_ptr<ModbusDiscreteIoClient> client;

    explicit ReconnectFixture(const unsigned retry_delay_ms = 0)
    {
        auto journal_copy = journal;

        auto s = settings();
        s.connection_retry_delay_ms = retry_delay_ms;

        client = std::make_unique<ModbusDiscreteIoClient>(
            [journal_copy]() -> std::unique_ptr<ModbusTransportPort> {
                journal_copy->transports_created++;

                if (journal_copy->factory_fails) {
                    throw std::runtime_error("connection refused");
                }

                return std::make_unique<FakeModbusTransport>(journal_copy);
            },
            s, logger);
    }
};

}  // namespace

TEST(ModbusDiscreteIoClientTest, AFailedOperationDropsTheTransport)
{
    ReconnectFixture f;
    ASSERT_TRUE(f.client->isConnected());
    ASSERT_EQ(f.journal->transports_created, 1);

    f.journal->throw_error = MB::utils::Timeout;
    EXPECT_THROW(f.client->readDiscreteCoil(CoilInfo{Coil::COIL_0, false, false}), MB::ModbusException);

    EXPECT_FALSE(f.client->isConnected());
    EXPECT_TRUE(f.journal->closed);
}

TEST(ModbusDiscreteIoClientTest, ReDialsAfterADropAndResumesWorking)
{
    // Zero configured delay so the backoff window has already elapsed by the next call.
    ReconnectFixture f;

    f.journal->throw_error = MB::utils::Timeout;
    EXPECT_THROW(f.client->readDiscreteCoil(CoilInfo{Coil::COIL_0, false, false}), MB::ModbusException);
    ASSERT_FALSE(f.client->isConnected());

    // Link comes back.
    f.journal->throw_error.reset();
    f.journal->coil_value = true;

    // Even a zero configured delay keeps a 1 ms floor, so that an unreachable host is not
    // re-dialled on literally every transaction. Retry until the window has passed rather than
    // sleeping a fixed amount, so this cannot flake either way.
    uint16_t value = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

    while (std::chrono::steady_clock::now() < deadline) {
        try {
            value = f.client->readDiscreteCoil(CoilInfo{Coil::COIL_0, false, false});
            break;
        } catch (const std::runtime_error &) {
            // Still inside the backoff window.
        }
    }

    EXPECT_EQ(value, 1U);
    EXPECT_TRUE(f.client->isConnected());
    EXPECT_EQ(f.journal->transports_created, 2) << "the client did not re-dial";
}

TEST(ModbusDiscreteIoClientTest, FailsFastInsideTheBackoffWindowInsteadOfDialling)
{
    // A long delay means the window is still open on the very next call. MB::TCP::Connection's
    // ::connect() is blocking and untimed, so dialling on every transaction against a dead host
    // would stall the caller repeatedly - the whole reason the attempts are rationed.
    ReconnectFixture f{60000};

    f.journal->throw_error = MB::utils::Timeout;
    EXPECT_THROW(f.client->readDiscreteCoil(CoilInfo{Coil::COIL_0, false, false}), MB::ModbusException);
    ASSERT_FALSE(f.client->isConnected());

    f.journal->throw_error.reset();

    const int created_before = f.journal->transports_created;
    EXPECT_THROW(f.client->readDiscreteCoil(CoilInfo{Coil::COIL_0, false, false}), std::runtime_error);
    EXPECT_EQ(f.journal->transports_created, created_before)
        << "a reconnection was attempted inside the backoff window";
}

TEST(ModbusDiscreteIoClientTest, AFailedReDialLeavesTheClientDisconnectedAndThrowing)
{
    ReconnectFixture f;

    f.journal->throw_error = MB::utils::Timeout;
    EXPECT_THROW(f.client->readDiscreteCoil(CoilInfo{Coil::COIL_0, false, false}), MB::ModbusException);

    f.journal->throw_error.reset();
    f.journal->factory_fails = true;

    EXPECT_THROW(f.client->readDiscreteCoil(CoilInfo{Coil::COIL_0, false, false}), std::runtime_error);
    EXPECT_FALSE(f.client->isConnected());
}

}  // namespace rover::transport::modbus::test
