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
// Modified 2026 by Mechatronics Academy: the ASIO-typed half of
// serial_driver/test/test_serial_port.cpp, rewritten against the free translation
// functions that replaced SerialPortConfig::get_*_asio().

#include <gtest/gtest.h>

#include "rover_serial_driver/infrastructure/asio_serial_options.hpp"

using rover::transport::serial::FlowControl;
using rover::transport::serial::Parity;
using rover::transport::serial::SerialPortBase;
using rover::transport::serial::SerialPortConfig;
using rover::transport::serial::StopBits;
using rover::transport::serial::toAsioBaudRate;
using rover::transport::serial::toAsioFlowControl;
using rover::transport::serial::toAsioParity;
using rover::transport::serial::toAsioStopBits;

TEST(AsioSerialOptionsTest, BaudRateSurvivesTranslation)
{
    const SerialPortConfig config{115200, FlowControl::NONE, Parity::NONE, StopBits::ONE};
    EXPECT_EQ(toAsioBaudRate(config).value(), SerialPortBase::baud_rate{115200}.value());
}

TEST(AsioSerialOptionsTest, EveryFlowControlMapsToItsAsioCounterpart)
{
    EXPECT_EQ(toAsioFlowControl(FlowControl::NONE), SerialPortBase::flow_control::none);
    EXPECT_EQ(toAsioFlowControl(FlowControl::HARDWARE), SerialPortBase::flow_control::hardware);
    EXPECT_EQ(toAsioFlowControl(FlowControl::SOFTWARE), SerialPortBase::flow_control::software);
}

TEST(AsioSerialOptionsTest, EveryParityMapsToItsAsioCounterpart)
{
    EXPECT_EQ(toAsioParity(Parity::NONE), SerialPortBase::parity::none);
    EXPECT_EQ(toAsioParity(Parity::ODD), SerialPortBase::parity::odd);
    EXPECT_EQ(toAsioParity(Parity::EVEN), SerialPortBase::parity::even);
}

TEST(AsioSerialOptionsTest, EveryStopBitsMapsToItsAsioCounterpart)
{
    EXPECT_EQ(toAsioStopBits(StopBits::ONE), SerialPortBase::stop_bits::one);
    EXPECT_EQ(
        toAsioStopBits(StopBits::ONE_POINT_FIVE), SerialPortBase::stop_bits::onepointfive);
    EXPECT_EQ(toAsioStopBits(StopBits::TWO), SerialPortBase::stop_bits::two);
}
