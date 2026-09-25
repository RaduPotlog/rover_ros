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
// infrastructure/mb_frame_mapping on its own: DiscreteRequest -> MB::ModbusRequest and
// MB::ModbusResponse -> DiscreteReply. No socket, no ROS.
//
// The parity test is the one that matters for the safety PLC link. Before ModbusTransportPort
// was narrowed, the client built each MB::ModbusRequest directly; it now builds a DiscreteRequest
// and this mapping builds the MB::ModbusRequest. Every request must be field-for-field and
// byte-for-byte what the direct construction produced.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include <MB/modbusCell.hpp>
#include <MB/modbusRequest.hpp>
#include <MB/modbusResponse.hpp>
#include <MB/modbusUtils.hpp>

#include "rover_modbus_driver/domain/discrete_transaction.hpp"
#include "rover_modbus_driver/infrastructure/mb_frame_mapping.hpp"

namespace rover::transport::modbus::test
{

using Bytes = std::vector<uint8_t>;

TEST(MbFrameMappingTest, FunctionEnumeratorsAreTheWireFunctionCodes)
{
    EXPECT_EQ(static_cast<uint8_t>(DiscreteFunction::READ_COILS), 0x01U);
    EXPECT_EQ(static_cast<uint8_t>(DiscreteFunction::READ_DISCRETE_INPUTS), 0x02U);
    EXPECT_EQ(static_cast<uint8_t>(DiscreteFunction::WRITE_SINGLE_COIL), 0x05U);

    EXPECT_EQ(
        static_cast<uint8_t>(DiscreteFunction::READ_COILS),
        static_cast<uint8_t>(MB::utils::ReadDiscreteOutputCoils));
    EXPECT_EQ(
        static_cast<uint8_t>(DiscreteFunction::READ_DISCRETE_INPUTS),
        static_cast<uint8_t>(MB::utils::ReadDiscreteInputContacts));
    EXPECT_EQ(
        static_cast<uint8_t>(DiscreteFunction::WRITE_SINGLE_COIL),
        static_cast<uint8_t>(MB::utils::WriteSingleDiscreteOutputCoil));
}

TEST(MbFrameMappingTest, ReadDiscreteInputsBecomesTheFc2Request)
{
    const MB::ModbusRequest req =
        toMbRequest(DiscreteRequest{255, DiscreteFunction::READ_DISCRETE_INPUTS, 3, 1, false});

    EXPECT_EQ(req.slaveID(), 255U);
    EXPECT_EQ(req.functionCode(), MB::utils::ReadDiscreteInputContacts);
    EXPECT_EQ(req.registerAddress(), 3U);
    EXPECT_EQ(req.numberOfRegisters(), 1U);
    EXPECT_TRUE(req.registerValues().empty());
    EXPECT_EQ(req.toRaw(), (Bytes{0xFF, 0x02, 0x00, 0x03, 0x00, 0x01}));
}

TEST(MbFrameMappingTest, ReadCoilsBecomesTheFc1Request)
{
    const MB::ModbusRequest req =
        toMbRequest(DiscreteRequest{255, DiscreteFunction::READ_COILS, 14, 6, false});

    EXPECT_EQ(req.functionCode(), MB::utils::ReadDiscreteOutputCoils);
    EXPECT_TRUE(req.registerValues().empty());
    EXPECT_EQ(req.toRaw(), (Bytes{0xFF, 0x01, 0x00, 0x0E, 0x00, 0x06}));
}

TEST(MbFrameMappingTest, WriteSingleCoilOnBecomesFf00)
{
    const MB::ModbusRequest req =
        toMbRequest(DiscreteRequest{255, DiscreteFunction::WRITE_SINGLE_COIL, 4, 1, true});

    EXPECT_EQ(req.functionCode(), MB::utils::WriteSingleDiscreteOutputCoil);
    ASSERT_EQ(req.registerValues().size(), 1U);
    ASSERT_TRUE(req.registerValues().front().isCoil());
    EXPECT_TRUE(req.registerValues().front().coil());
    EXPECT_EQ(req.toRaw(), (Bytes{0xFF, 0x05, 0x00, 0x04, 0xFF, 0x00}));
}

TEST(MbFrameMappingTest, WriteSingleCoilOffBecomes0000)
{
    const MB::ModbusRequest req =
        toMbRequest(DiscreteRequest{255, DiscreteFunction::WRITE_SINGLE_COIL, 4, 1, false});

    ASSERT_EQ(req.registerValues().size(), 1U);
    EXPECT_FALSE(req.registerValues().front().coil());
    EXPECT_EQ(req.toRaw(), (Bytes{0xFF, 0x05, 0x00, 0x04, 0x00, 0x00}));
}

// The MB::ModbusRequest constructions below are exactly what ModbusDiscreteIoClient did before
// the port was narrowed (unit id 255, the function code, the address, the count, and for FC5 a
// single coil cell).
TEST(MbFrameMappingTest, EveryRequestIsByteIdenticalToTheDirectlyBuiltMbRequest)
{
    const std::vector<uint16_t> addresses = {0, 5, 8, 19, 0xFFFF};
    const std::vector<uint16_t> counts    = {1, 6, 8, 12};

    const auto expectSame = [](const MB::ModbusRequest & mapped, const MB::ModbusRequest & direct) {
        EXPECT_EQ(mapped.slaveID(), direct.slaveID());
        EXPECT_EQ(mapped.functionCode(), direct.functionCode());
        EXPECT_EQ(mapped.registerAddress(), direct.registerAddress());
        EXPECT_EQ(mapped.numberOfRegisters(), direct.numberOfRegisters());
        EXPECT_EQ(mapped.toRaw(), direct.toRaw());
    };

    int compared = 0;

    for (const auto function :
        {DiscreteFunction::READ_COILS, DiscreteFunction::READ_DISCRETE_INPUTS})
    {
        const auto fc = function == DiscreteFunction::READ_COILS
            ? MB::utils::ReadDiscreteOutputCoils
            : MB::utils::ReadDiscreteInputContacts;

        for (const uint16_t address : addresses) {
            for (const uint16_t count : counts) {
                SCOPED_TRACE(
                    "fc " + std::to_string(fc) + " address " + std::to_string(address) +
                    " count " + std::to_string(count));

                expectSame(
                    toMbRequest(DiscreteRequest{255, function, address, count, false}),
                    MB::ModbusRequest(255, fc, address, count));
                ++compared;
            }
        }
    }

    for (const uint16_t address : addresses) {
        for (const bool value : {false, true}) {
            SCOPED_TRACE(
                "fc 5 address " + std::to_string(address) + " value " + std::to_string(value));

            const MB::ModbusRequest mapped = toMbRequest(
                DiscreteRequest{255, DiscreteFunction::WRITE_SINGLE_COIL, address, 1, value});
            const MB::ModbusRequest direct(
                255, MB::utils::WriteSingleDiscreteOutputCoil, address, 1,
                std::vector<MB::ModbusCell>{MB::ModbusCell(value)});

            expectSame(mapped, direct);
            ASSERT_EQ(mapped.registerValues().size(), 1U);
            EXPECT_EQ(mapped.registerValues().front().coil(), value);
            ++compared;
        }
    }

    EXPECT_EQ(compared, 2 * 5 * 4 + 5 * 2);
}

// A real device answers a coil read in whole bytes: byte 0x01 then 0x08 is 16 cells, bit 0 and
// bit 11 set. Every cell comes through, padding included, in address order.
TEST(MbFrameMappingTest, CoilReplyCellsKeepTheirOrderAndPadding)
{
    const DiscreteReply reply =
        toDiscreteReply(MB::ModbusResponse::fromRaw({0xFF, 0x01, 0x02, 0x01, 0x08}));

    ASSERT_EQ(reply.cells.size(), 16U);

    for (const ReplyCell & cell : reply.cells) {
        EXPECT_TRUE(cell.is_coil);
    }

    EXPECT_TRUE(reply.cells[0].value);
    EXPECT_FALSE(reply.cells[1].value);
    EXPECT_TRUE(reply.cells[11].value);
    EXPECT_FALSE(reply.cells[15].value);
}

// MB::ModbusResponse::registerValues() throws on a reply with no values. The mapping turns that
// into an empty reply instead, so the transport returns it and the client rejects it without
// dropping the link.
TEST(MbFrameMappingTest, AReplyWithNoValuesBecomesAnEmptyReplyWithoutThrowing)
{
    DiscreteReply reply{};

    EXPECT_NO_THROW(
        reply = toDiscreteReply(
            MB::ModbusResponse(0xFF, MB::utils::ReadDiscreteInputContacts, 0, 1, {})));
    EXPECT_TRUE(reply.cells.empty());

    reply = DiscreteReply{{ReplyCell{true, true}}};

    EXPECT_NO_THROW(reply = toDiscreteReply(MB::ModbusResponse::fromRaw({0xFF, 0x02, 0x00})));
    EXPECT_TRUE(reply.cells.empty());
}

TEST(MbFrameMappingTest, RegisterCellsBecomeNonCoilCells)
{
    const DiscreteReply reply =
        toDiscreteReply(MB::ModbusResponse::fromRaw({0xFF, 0x03, 0x02, 0x00, 0x01}));

    ASSERT_EQ(reply.cells.size(), 1U);
    EXPECT_FALSE(reply.cells.front().is_coil);
}

}  // namespace rover::transport::modbus::test
