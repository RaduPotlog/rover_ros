// Copyright 2025 Mechatronics Academy
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
// Unit tests for ContactCoilHandler, run against a FakeRoverModbus (no real Modbus TCP
// connection). Assertions are restricted to what's deterministic without synchronizing on the
// background poll thread (see .claude/rules/testing.md's "never sleep(N) to synchronize"):
// the coil-trigger methods are synchronous themselves, and initCoils() runs synchronously inside
// start() before the poll thread is spawned, so both are safe to assert on immediately.

#include <gtest/gtest.h>

#include <memory>

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller.hpp"

#include "fake_rover_modbus.hpp"

namespace rover_hardware_interface
{
namespace test
{

class ContactCoilHandlerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        modbus = std::make_shared<FakeRoverModbus>();
        handler = std::make_unique<ContactCoilHandler>(modbus);
    }

    std::shared_ptr<FakeRoverModbus> modbus;
    std::unique_ptr<ContactCoilHandler> handler;
};

TEST_F(ContactCoilHandlerTest, StartsDisabled)
{
    EXPECT_FALSE(handler->isContactCoilHandlerEnabled());
}

TEST_F(ContactCoilHandlerTest, EStopUserBtnTriggerWritesCoil2)
{
    handler->eStopUserBtnTrigger(true);

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_2, true}));
}

TEST_F(ContactCoilHandlerTest, EStopMotorDriverFaultTriggerWritesCoil3)
{
    handler->eStopMotorDriverFaultTrigger(true);

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_3, true}));
}

TEST_F(ContactCoilHandlerTest, EStopLatchResetPulsesCoil4TrueThenFalse)
{
    handler->eStopLatchReset();

    const auto writes = modbus->writesSnapshot();
    ASSERT_EQ(writes.size(), 2u);
    EXPECT_EQ(writes[0], (CoilWrite{Coil::COIL_4, true}));
    EXPECT_EQ(writes[1], (CoilWrite{Coil::COIL_4, false}));
}

TEST_F(ContactCoilHandlerTest, StartWritesEachCoilsDefaultStateBeforeReturning)
{
    ASSERT_TRUE(handler->start());
    EXPECT_TRUE(handler->isContactCoilHandlerEnabled());

    // Matches coils_config_info_storage_'s default_coil_state for each coil (see
    // rover_safety_controller.cpp) - initCoils() runs synchronously inside start(), before the
    // poll thread is spawned, so this is deterministic.
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_0, false}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_1, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_2, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_3, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_4, false}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_5, false}));
}

TEST_F(ContactCoilHandlerTest, StartIsIdempotent)
{
    ASSERT_TRUE(handler->start());
    EXPECT_TRUE(handler->start());
    EXPECT_TRUE(handler->isContactCoilHandlerEnabled());
}

}  // namespace test
}  // namespace rover_hardware_interface
