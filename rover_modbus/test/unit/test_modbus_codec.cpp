// Copyright 2026 Mechatronics Academy
// Licensed under: MIT License <http://opensource.org/licenses/MIT>

//
// The frame codec on its own: CRC, request encoding, response and exception decoding. Links
// Modbus_Core only - no sockets.
//
// Pins the short-reply guard in ModbusResponse (README, "Local changes to this fork") and the
// behaviour rover_modbus_driver relies on: the request bytes it puts on the wire, and
// registerValues() throwing exactly when a reply carries no values.
//
// <MB/crc.hpp> has no include guard, so it is never included directly here: MB::CRC comes in
// once, through <MB/modbusUtils.hpp>.

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <vector>

#include <MB/modbusCell.hpp>
#include <MB/modbusException.hpp>
#include <MB/modbusRequest.hpp>
#include <MB/modbusResponse.hpp>
#include <MB/modbusUtils.hpp>

namespace
{

// Appends the CRC low byte first, as the README prints it (0xfd 0xd2 for 0xD2FD). fromRawCRC()
// reads the trailing two bytes back as a native-endian uint16_t.
std::vector<uint8_t> withCrc(std::vector<uint8_t> frame)
{
    const uint16_t crc = MB::CRC::calculateCRC(frame);
    frame.push_back(static_cast<uint8_t>(crc & 0xFFu));
    frame.push_back(static_cast<uint8_t>(crc >> 8));

    return frame;
}

// The error code of the MB::ModbusException `call` throws, or nullopt if it throws none.
// Compared by code, never by what(): what() copies into a static buffer without a NUL.
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

}  // namespace

TEST(CrcTest, MatchesTheReadmeExample)
{
    EXPECT_EQ(MB::CRC::calculateCRC(std::vector<uint8_t>{0x01, 0x01, 0x00, 0x64, 0x00, 0x0A}),
              0xD2FD);
}

TEST(CrcTest, MatchesTheModbusReferenceFrame)
{
    EXPECT_EQ(MB::CRC::calculateCRC(std::vector<uint8_t>{0x01, 0x03, 0x00, 0x00, 0x00, 0x0A}),
              0xCDC5);
}

TEST(CrcTest, LengthArgumentLimitsTheBytesCovered)
{
    const std::vector<uint8_t> buffer{0x01, 0x01, 0x00, 0x64, 0x00, 0x0A, 0xAA, 0xBB};

    EXPECT_EQ(MB::CRC::calculateCRC(buffer, 6), 0xD2FD);
}

TEST(CrcTest, ARequestRoundTripsThroughFromRawCrc)
{
    const auto raw =
        withCrc(MB::ModbusRequest(1, MB::utils::ReadDiscreteOutputCoils, 100, 10).toRaw());

    ASSERT_EQ(raw.size(), 8U);
    EXPECT_EQ(raw[6], 0xFD);
    EXPECT_EQ(raw[7], 0xD2);

    const auto request = MB::ModbusRequest::fromRawCRC(raw);

    EXPECT_EQ(request.slaveID(), 1);
    EXPECT_EQ(request.functionCode(), MB::utils::ReadDiscreteOutputCoils);
    EXPECT_EQ(request.registerAddress(), 100);
    EXPECT_EQ(request.numberOfRegisters(), 10);
}

TEST(CrcTest, ACorruptedRequestCrcIsRejected)
{
    auto raw = withCrc(MB::ModbusRequest(1, MB::utils::ReadDiscreteOutputCoils, 100, 10).toRaw());
    raw[3] ^= 0xFF;

    EXPECT_EQ(thrownErrorCode([&] { (void)MB::ModbusRequest::fromRawCRC(raw); }),
              MB::utils::InvalidCRC);
}

TEST(CrcTest, ACorruptedResponseCrcIsRejected)
{
    auto raw = withCrc({0xFF, 0x01, 0x01, 0x05});

    const auto response = MB::ModbusResponse::fromRawCRC(raw);
    const auto & values = response.registerValues();

    ASSERT_EQ(values.size(), 8U);
    for (std::size_t i = 0; i < values.size(); ++i) {
        EXPECT_EQ(values[i].coil(), i == 0 || i == 2) << "coil " << i;
    }

    raw[3] ^= 0x02;

    EXPECT_EQ(thrownErrorCode([&] { (void)MB::ModbusResponse::fromRawCRC(raw); }),
              MB::utils::InvalidCRC);
}

TEST(RequestEncodingTest, ReadRequestIsSlaveFunctionAddressCountBigEndian)
{
    const MB::ModbusRequest request(0xFF, MB::utils::ReadDiscreteInputContacts, 0x0102, 0x0304);

    EXPECT_EQ(request.toRaw(), (std::vector<uint8_t>{0xFF, 0x02, 0x01, 0x02, 0x03, 0x04}));
}

TEST(RequestEncodingTest, WriteSingleCoilIsFf00OrZero)
{
    const MB::ModbusRequest on(0xFF, MB::utils::WriteSingleDiscreteOutputCoil, 4, 1,
                               {MB::ModbusCell::initCoil(true)});
    const MB::ModbusRequest off(0xFF, MB::utils::WriteSingleDiscreteOutputCoil, 4, 1,
                                {MB::ModbusCell::initCoil(false)});

    EXPECT_EQ(on.toRaw(), (std::vector<uint8_t>{0xFF, 0x05, 0x00, 0x04, 0xFF, 0x00}));
    EXPECT_EQ(off.toRaw(), (std::vector<uint8_t>{0xFF, 0x05, 0x00, 0x04, 0x00, 0x00}));
}

// The short-reply guard: a reply whose byte count promises more data than it carries used to be
// decoded from past the end of the buffer.
TEST(ResponseDecodingTest, ACoilReplyShorterThanItsByteCountIsRejected)
{
    EXPECT_EQ(thrownErrorCode([] { (void)MB::ModbusResponse::fromRaw({0xFF, 0x01, 0x02, 0x00}); }),
              MB::utils::NumberOfValuesInvalid);
}

TEST(ResponseDecodingTest, AContactReplyShorterThanItsByteCountIsRejected)
{
    EXPECT_EQ(thrownErrorCode([] { (void)MB::ModbusResponse::fromRaw({0xFF, 0x02, 0x02, 0x00}); }),
              MB::utils::NumberOfValuesInvalid);
}

TEST(ResponseDecodingTest, ARegisterReplyShorterThanItsByteCountIsRejected)
{
    EXPECT_EQ(
        thrownErrorCode([] { (void)MB::ModbusResponse::fromRaw({0xFF, 0x03, 0x04, 0x00, 0x01}); }),
        MB::utils::NumberOfValuesInvalid);
}

TEST(ResponseDecodingTest, ACoilReplyDecodesEightCellsPerByteLsbFirst)
{
    const auto response = MB::ModbusResponse::fromRaw({0xFF, 0x01, 0x02, 0x01, 0x08});
    const auto & values = response.registerValues();

    ASSERT_EQ(values.size(), 16U);
    EXPECT_TRUE(values[0].coil());
    EXPECT_FALSE(values[1].coil());
    EXPECT_TRUE(values[11].coil());
    EXPECT_FALSE(values[15].coil());
}

// Decoding an empty reply succeeds (a throw here fails the test); only asking for its values
// throws. rover_modbus_driver relies on exactly that to reject an empty reply.
TEST(ResponseDecodingTest, AZeroByteCountReplyHasNoValues)
{
    const auto response = MB::ModbusResponse::fromRaw({0xFF, 0x02, 0x00});

    EXPECT_EQ(thrownErrorCode([&] { (void)response.registerValues(); }),
              MB::utils::NumberOfValuesInvalid);
}

TEST(ResponseDecodingTest, AFrameShorterThanThreeBytesIsInvalidByteOrder)
{
    EXPECT_EQ(thrownErrorCode([] { (void)MB::ModbusResponse::fromRaw({0xFF, 0x01}); }),
              MB::utils::InvalidByteOrder);
}

TEST(ExceptionDecodingTest, AnExceptionPduCarriesSlaveFunctionAndCode)
{
    const std::vector<uint8_t> pdu{0xFF, 0x82, 0x02};

    EXPECT_TRUE(MB::ModbusException::exist(pdu));

    const MB::ModbusException exception(pdu);

    EXPECT_EQ(exception.slaveID(), 255);
    EXPECT_EQ(exception.functionCode(), MB::utils::ReadDiscreteInputContacts);
    EXPECT_EQ(exception.getErrorCode(), MB::utils::IllegalDataAddress);
    EXPECT_EQ(exception.toString(),
              "Error on slave 255 - Illegal data address ( on function: Read from input contacts )");
}
