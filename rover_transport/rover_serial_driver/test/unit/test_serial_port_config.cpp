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
// Modified 2026 by Mechatronics Academy: the internally-typed half of
// serial_driver/test/test_serial_port.cpp (ros-drivers/transport_drivers v1.2.0), plus new
// coverage for the parameter parsing that upstream had inline in the node.

#include <gtest/gtest.h>

#include <string>

#include "rover_serial_driver/domain/serial_port_config.hpp"

using rover::transport::serial::FlowControl;
using rover::transport::serial::Parity;
using rover::transport::serial::SerialPortConfig;
using rover::transport::serial::StopBits;
using rover::transport::serial::flowControlFromString;
using rover::transport::serial::parityFromString;
using rover::transport::serial::stopBitsFromString;

TEST(SerialPortConfigTest, HoldsWhatItWasGiven)
{
    const SerialPortConfig config{115200, FlowControl::NONE, Parity::NONE, StopBits::ONE};

    EXPECT_EQ(config.getBaudRate(), 115200u);
    EXPECT_EQ(config.getFlowControl(), FlowControl::NONE);
    EXPECT_EQ(config.getParity(), Parity::NONE);
    EXPECT_EQ(config.getStopBits(), StopBits::ONE);
}

TEST(SerialPortConfigTest, ParsesEveryAcceptedFlowControlSpelling)
{
    EXPECT_EQ(flowControlFromString("none"), FlowControl::NONE);
    EXPECT_EQ(flowControlFromString("hardware"), FlowControl::HARDWARE);
    EXPECT_EQ(flowControlFromString("software"), FlowControl::SOFTWARE);
    EXPECT_FALSE(flowControlFromString("").has_value());
    EXPECT_FALSE(flowControlFromString("NONE").has_value());
    EXPECT_FALSE(flowControlFromString("rts").has_value());
}

TEST(SerialPortConfigTest, ParsesEveryAcceptedParitySpelling)
{
    EXPECT_EQ(parityFromString("none"), Parity::NONE);
    EXPECT_EQ(parityFromString("odd"), Parity::ODD);
    EXPECT_EQ(parityFromString("even"), Parity::EVEN);
    EXPECT_FALSE(parityFromString("mark").has_value());
}

TEST(SerialPortConfigTest, ParsesEveryAcceptedStopBitsSpelling)
{
    // Both "1" and "1.0" are accepted upstream; so are "2" and "2.0". Keep it that way -
    // rover_crsf_teleop's config relies on the quoted "1" form.
    EXPECT_EQ(stopBitsFromString("1"), StopBits::ONE);
    EXPECT_EQ(stopBitsFromString("1.0"), StopBits::ONE);
    EXPECT_EQ(stopBitsFromString("1.5"), StopBits::ONE_POINT_FIVE);
    EXPECT_EQ(stopBitsFromString("2"), StopBits::TWO);
    EXPECT_EQ(stopBitsFromString("2.0"), StopBits::TWO);
    EXPECT_FALSE(stopBitsFromString("3").has_value());
    EXPECT_FALSE(stopBitsFromString("").has_value());
}

TEST(SerialPortConfigTest, FromStringsAcceptsAValidCombination)
{
    std::string error;
    const auto config = SerialPortConfig::fromStrings(460800, "none", "none", "1", error);

    ASSERT_TRUE(config.has_value());
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(config->getBaudRate(), 460800u);
    EXPECT_EQ(config->getStopBits(), StopBits::ONE);
}

TEST(SerialPortConfigTest, FromStringsRejectsNonPositiveBaudRate)
{
    // Upstream defaulted baud_rate to 0 and never checked it, so a missing parameter only
    // failed later, as an opaque ASIO error at open() time.
    std::string error;
    EXPECT_FALSE(SerialPortConfig::fromStrings(0, "none", "none", "1", error).has_value());
    EXPECT_NE(error.find("baud_rate"), std::string::npos);

    error.clear();
    EXPECT_FALSE(SerialPortConfig::fromStrings(-1, "none", "none", "1", error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(SerialPortConfigTest, FromStringsReportsWhichFieldWasBad)
{
    std::string error;

    EXPECT_FALSE(SerialPortConfig::fromStrings(115200, "rts", "none", "1", error).has_value());
    EXPECT_NE(error.find("flow_control"), std::string::npos);

    error.clear();
    EXPECT_FALSE(SerialPortConfig::fromStrings(115200, "none", "mark", "1", error).has_value());
    EXPECT_NE(error.find("parity"), std::string::npos);

    error.clear();
    EXPECT_FALSE(SerialPortConfig::fromStrings(115200, "none", "none", "3", error).has_value());
    EXPECT_NE(error.find("stop_bits"), std::string::npos);
}
