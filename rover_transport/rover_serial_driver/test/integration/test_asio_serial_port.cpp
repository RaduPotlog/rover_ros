// Copyright 2021 LeoDrive, Copyright 2021 The Autoware Foundation
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
// Modified 2026 by Mechatronics Academy: the state / send-break half of
// serial_driver/test/test_serial_port.cpp (ros-drivers/transport_drivers v1.2.0).
//
// These need no real UART: /dev/ttyS0 is constructed but never opened (ASIO's serial_port
// constructor does not touch the device), and opening /dev/null is expected to throw. The
// one case that does need a device is skipped unless ROVER_SERIAL_TEST_DEV is set -
// upstream had it commented out, which hid it from anyone with hardware attached.

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <vector>

#include "rover_serial_driver/infrastructure/asio_serial_port.hpp"

using rover::transport::IoContext;
using rover::transport::serial::AsioSerialPort;
using rover::transport::serial::FlowControl;
using rover::transport::serial::Parity;
using rover::transport::serial::SerialPortConfig;
using rover::transport::serial::StopBits;

namespace
{

constexpr const char * kDevName = "/dev/ttyS0";
constexpr const char * kDevNull = "/dev/null";
constexpr uint32_t kBaud = 115200;

SerialPortConfig defaultConfig()
{
    return SerialPortConfig{kBaud, FlowControl::NONE, Parity::NONE, StopBits::ONE};
}

}  // namespace

TEST(AsioSerialPortTest, PropertiesTest)
{
    IoContext ctx;
    AsioSerialPort port(ctx, kDevName, defaultConfig());

    EXPECT_EQ(port.deviceName(), kDevName);
    EXPECT_EQ(port.serialPortConfig().getBaudRate(), kBaud);
    EXPECT_EQ(port.serialPortConfig().getFlowControl(), FlowControl::NONE);
    EXPECT_EQ(port.serialPortConfig().getParity(), Parity::NONE);
    EXPECT_EQ(port.serialPortConfig().getStopBits(), StopBits::ONE);

    ctx.waitForExit();
}

TEST(AsioSerialPortTest, StateTest)
{
    IoContext ctx;
    AsioSerialPort port(ctx, kDevName, defaultConfig());

    std::vector<uint8_t> send_recv_buff;

    EXPECT_FALSE(port.isOpen());
    EXPECT_THROW(port.send(send_recv_buff), asio::system_error);
    EXPECT_THROW(port.receive(send_recv_buff), asio::system_error);

    ctx.waitForExit();
}

TEST(AsioSerialPortTest, SendBreakWhileClosed)
{
    IoContext ctx;
    AsioSerialPort port(ctx, kDevNull, defaultConfig());

    EXPECT_FALSE(port.isOpen());
    EXPECT_FALSE(port.sendBreak());

    ctx.waitForExit();
}

TEST(AsioSerialPortTest, SendBreakWhenFailedToOpen)
{
    IoContext ctx;
    AsioSerialPort port(ctx, kDevNull, defaultConfig());

    EXPECT_FALSE(port.isOpen());
    EXPECT_THROW(port.open(), asio::system_error);
    EXPECT_FALSE(port.isOpen());
    EXPECT_FALSE(port.sendBreak());

    ctx.waitForExit();
}

TEST(AsioSerialPortTest, SendBreakWhileOpen)
{
    const char * device = std::getenv("ROVER_SERIAL_TEST_DEV");
    if (device == nullptr) {
        GTEST_SKIP() << "Set ROVER_SERIAL_TEST_DEV to a real serial device to run this.";
    }

    IoContext ctx;
    AsioSerialPort port(ctx, device, defaultConfig());

    EXPECT_FALSE(port.isOpen());
    EXPECT_NO_THROW(port.open());
    EXPECT_TRUE(port.isOpen());
    EXPECT_TRUE(port.sendBreak());

    ctx.waitForExit();
}
