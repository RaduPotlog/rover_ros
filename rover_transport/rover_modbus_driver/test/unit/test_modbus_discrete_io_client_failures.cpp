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
// ModbusDiscreteIoClient's failure paths, one decision at a time: which exception comes out,
// what is logged, and whether the link is kept or dropped.
//
// The distinction that matters on the safety path is WHERE a bad reply is rejected. A reply
// that arrived but carries no values, too few values, or non-coil cells is rejected by the
// client after the transaction, so the link is kept. Anything the transport itself throws
// (timeout, closed socket, a device exception PDU) drops the link and re-dials later. These
// tests pin that split, together with the exact log lines, so a change to the transport seam
// cannot move a failure from one side to the other unnoticed.

#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <MB/modbusException.hpp>
#include <MB/modbusUtils.hpp>

#include "fakes/fake_logger.hpp"
#include "fakes/fake_modbus_transport.hpp"
#include "rover_modbus_driver/application/modbus_discrete_io_client.hpp"

namespace rover::transport::modbus::test
{

namespace
{

// The MB::ModbusException `op` throws, or nullopt if it returns normally. Any other exception
// propagates and fails the test.
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

// A client over the fake transport whose factory counts its calls, so a re-dial (or its
// absence) is observable.
struct FailureFixture
{
    std::shared_ptr<Journal> journal{std::make_shared<Journal>()};
    std::shared_ptr<FakeLogger> logger{std::make_shared<FakeLogger>()};
    std::unique_ptr<ModbusDiscreteIoClient> client;

    explicit FailureFixture(const unsigned retry_delay_ms = 0)
    {
        auto journal_copy = journal;

        ClientSettings s;
        s.host                      = "127.0.0.1";
        s.port                      = 502;
        s.connection_retry_count    = 1;
        s.connection_retry_delay_ms = retry_delay_ms;

        client = std::make_unique<ModbusDiscreteIoClient>(
            [journal_copy]() -> std::unique_ptr<ModbusTransportPort> {
                journal_copy->transports_created++;
                return std::make_unique<FakeModbusTransport>(journal_copy);
            },
            s, logger);
    }

    // Fails one transaction in the transport so the client drops the link, then lets the
    // transport answer again. With a long retry delay the client stays inside the backoff
    // window afterwards.
    void dropTheLink()
    {
        journal->throw_error = MB::utils::Timeout;
        EXPECT_TRUE(modbusExceptionFrom([this]() {
            client->readDiscreteCoil(CoilInfo{Coil::COIL_0, false, false});
        }).has_value());
        journal->throw_error.reset();

        ASSERT_FALSE(client->isConnected());
    }
};

}  // namespace

// --- Replies rejected by the client: the link is kept ------------------------------------

TEST(ModbusDiscreteIoClientFailureTest, AnEmptyContactReplyThrowsNumberOfValuesInvalidAndKeepsTheLink)
{
    FailureFixture f;
    f.journal->reply_empty = true;

    const auto e = modbusExceptionFrom([&f]() {
        f.client->readDiscreteContact(ContactInfo{Contact::CONTACT_0});
    });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::NumberOfValuesInvalid);
    EXPECT_EQ(e->slaveID(), 255U);
    EXPECT_EQ(e->functionCode(), MB::utils::Undefined);

    EXPECT_EQ(f.logger->errors, (std::vector<std::string>{"Failed to read contact"}));
    EXPECT_TRUE(f.logger->warnings.empty());
    EXPECT_TRUE(f.client->isConnected());
    EXPECT_FALSE(f.journal->closed);
    EXPECT_EQ(f.journal->transports_created, 1);
}

TEST(ModbusDiscreteIoClientFailureTest, AnEmptyCoilReplyLogsFailedToReadCoil)
{
    FailureFixture f;
    f.journal->reply_empty = true;

    const auto e = modbusExceptionFrom([&f]() {
        f.client->readDiscreteCoil(CoilInfo{Coil::COIL_5, false, false});
    });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::NumberOfValuesInvalid);

    EXPECT_EQ(f.logger->errors, (std::vector<std::string>{"Failed to read coil"}));
    EXPECT_TRUE(f.client->isConnected());
    EXPECT_EQ(f.journal->transports_created, 1);
}

TEST(ModbusDiscreteIoClientFailureTest, AnEmptyBatchedReplyThrowsWithoutAFunctionCode)
{
    FailureFixture f;
    f.journal->reply_empty = true;

    const auto e = modbusExceptionFrom([&f]() { f.client->readDiscreteCoils(Coil::COIL_0, 1); });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::NumberOfValuesInvalid);
    EXPECT_EQ(e->slaveID(), 255U);
    EXPECT_EQ(e->functionCode(), MB::utils::Undefined);

    EXPECT_EQ(f.logger->errors, (std::vector<std::string>{"Failed to read coils"}));
    EXPECT_TRUE(f.client->isConnected());
    EXPECT_EQ(f.journal->transports_created, 1);
}

TEST(ModbusDiscreteIoClientFailureTest, AShortBatchedCoilReplyCarriesFc1AndKeepsTheLink)
{
    FailureFixture f;
    f.journal->bit_values = std::vector<bool>{true, true, true};

    const auto e = modbusExceptionFrom([&f]() { f.client->readDiscreteCoils(Coil::COIL_0, 20); });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::NumberOfValuesInvalid);
    EXPECT_EQ(e->slaveID(), 255U);
    EXPECT_EQ(e->functionCode(), MB::utils::ReadDiscreteOutputCoils);

    EXPECT_EQ(f.logger->errors, (std::vector<std::string>{"Failed to read coils"}));
    EXPECT_TRUE(f.logger->warnings.empty());
    EXPECT_TRUE(f.client->isConnected());
    EXPECT_EQ(f.journal->transports_created, 1);
}

TEST(ModbusDiscreteIoClientFailureTest, AShortBatchedContactReplyCarriesFc2)
{
    FailureFixture f;
    f.journal->bit_values = std::vector<bool>{true};

    const auto e = modbusExceptionFrom([&f]() {
        f.client->readDiscreteContacts(Contact::CONTACT_0, 5);
    });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::NumberOfValuesInvalid);
    EXPECT_EQ(e->slaveID(), 255U);
    EXPECT_EQ(e->functionCode(), MB::utils::ReadDiscreteInputContacts);

    EXPECT_EQ(f.logger->errors, (std::vector<std::string>{"Failed to read contacts"}));
    EXPECT_TRUE(f.client->isConnected());
    EXPECT_EQ(f.journal->transports_created, 1);
}

TEST(ModbusDiscreteIoClientFailureTest, AZeroCountBatchedReadWithValuesReturnsNothing)
{
    FailureFixture f;
    f.journal->bit_values = std::vector<bool>{true};

    EXPECT_EQ(f.client->readDiscreteCoils(Coil::COIL_0, 0), std::vector<bool>{});

    ASSERT_EQ(f.journal->requests.size(), 1U);
    EXPECT_EQ(f.journal->requests.front().numberOfRegisters(), 0U);

    EXPECT_TRUE(f.logger->errors.empty());
    EXPECT_TRUE(f.client->isConnected());
    EXPECT_EQ(f.journal->transports_created, 1);
}

// --- Failures thrown by the transport: the link is dropped -------------------------------

TEST(ModbusDiscreteIoClientFailureTest, ATransportModbusExceptionIsLoggedDroppedAndRethrown)
{
    FailureFixture f;
    f.journal->throw_error = MB::utils::Timeout;

    const auto e = modbusExceptionFrom([&f]() {
        f.client->readDiscreteContact(ContactInfo{Contact::CONTACT_0});
    });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::Timeout);

    ASSERT_EQ(f.logger->errors.size(), 2U);
    EXPECT_EQ(
        f.logger->errors[0].rfind("Modbus exception: Error on slave 255 - Timeout", 0), 0U)
        << f.logger->errors[0];
    EXPECT_EQ(f.logger->errors[1], "Failed to read contact");

    ASSERT_EQ(f.logger->warnings.size(), 1U);
    EXPECT_NE(
        f.logger->warnings[0].find("dropped; next reconnection attempt in"), std::string::npos)
        << f.logger->warnings[0];

    EXPECT_FALSE(f.client->isConnected());
    EXPECT_TRUE(f.journal->closed);
    EXPECT_EQ(f.journal->transports_created, 1);
}

TEST(ModbusDiscreteIoClientFailureTest, AWriteFailureIsLoggedDroppedAndRethrown)
{
    FailureFixture f;
    f.journal->throw_error = MB::utils::Timeout;

    const auto e = modbusExceptionFrom([&f]() {
        f.client->writeDiscreteCoil(CoilInfo{Coil::COIL_1, false, true}, true);
    });

    ASSERT_TRUE(e.has_value());
    EXPECT_EQ(e->getErrorCode(), MB::utils::Timeout);

    ASSERT_EQ(f.logger->errors.size(), 2U);
    EXPECT_EQ(f.logger->errors[1], "Failed to write coil");

    EXPECT_FALSE(f.client->isConnected());
    EXPECT_TRUE(f.journal->closed);
}

// --- Failures before any transaction -----------------------------------------------------

// The engage guard runs before the link is even looked at: a refused write never dials, and
// never reports "link down" instead of the refusal.
TEST(ModbusDiscreteIoClientFailureTest, AWriteToANonEngageableCoilIsRefusedEvenWhenDisconnected)
{
    FailureFixture f{60000};
    f.dropTheLink();

    const int created_before   = f.journal->transports_created;
    const auto requests_before = f.journal->requests.size();

    EXPECT_NO_THROW(f.client->writeDiscreteCoil(CoilInfo{Coil::COIL_0, false, false}, true));

    ASSERT_FALSE(f.logger->errors.empty());
    EXPECT_EQ(f.logger->errors.back(), "Coil engage is not allowed");
    EXPECT_EQ(f.journal->transports_created, created_before);
    EXPECT_EQ(f.journal->requests.size(), requests_before);
    EXPECT_FALSE(f.client->isConnected());
}

// Inside the backoff window the client fails fast with std::runtime_error. That is not an
// MB::ModbusException, so the per-operation "Failed to read ..." line is not logged for it.
TEST(ModbusDiscreteIoClientFailureTest, InsideTheBackoffWindowFailuresAreNotLoggedAsReadFailures)
{
    FailureFixture f{60000};
    f.dropTheLink();

    const auto errors_before = f.logger->errors.size();

    try {
        f.client->readDiscreteContact(ContactInfo{Contact::CONTACT_0});
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error & e) {
        EXPECT_NE(
            std::string(e.what()).find("waiting out the reconnection backoff"), std::string::npos)
            << e.what();
    }

    EXPECT_EQ(f.logger->errors.size(), errors_before);
    EXPECT_EQ(f.journal->transports_created, 1);
}

}  // namespace rover::transport::modbus::test
