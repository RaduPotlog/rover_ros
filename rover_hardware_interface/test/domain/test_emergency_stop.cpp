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

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <vector>

#include "rover_hardware_interface/domain/emergency_stop.hpp"

namespace rover_hardware_interface
{

class FakeEmergencyStopIo : public EmergencyStopIoPort
{

public:

    bool isUserButtonActive() override { return user_button_active; }

    bool isLatchActive() override { return latch_active; }

    void triggerUserButton(const bool state) override
    {
        trigger_user_button_calls.push_back(state);
        user_button_active = state;
    }

    void resetLatch() override
    {
        reset_latch_calls++;
        latch_active = false;
    }

    bool user_button_active = true;
    bool latch_active = true;
    std::vector<bool> trigger_user_button_calls;
    unsigned reset_latch_calls = 0;
};

class EmergencyStopTest : public ::testing::Test
{
protected:
    void makeEStop(std::function<bool()> zero_velocity_check)
    {
        io = std::make_shared<FakeEmergencyStopIo>();
        e_stop = std::make_unique<EmergencyStop>(io, zero_velocity_check);
    }

    std::shared_ptr<FakeEmergencyStopIo> io;
    std::unique_ptr<EmergencyStop> e_stop;
};

TEST_F(EmergencyStopTest, ReadStateReflectsPortValues)
{
    makeEStop([]() { return true; });

    io->user_button_active = false;
    io->latch_active = true;

    EXPECT_FALSE(e_stop->readEStopState());
    EXPECT_TRUE(e_stop->readEStopLatchState());
}

TEST_F(EmergencyStopTest, SetEStopTriggersUserButton)
{
    makeEStop([]() { return true; });

    e_stop->setEStop();

    ASSERT_EQ(io->trigger_user_button_calls.size(), 1u);
    EXPECT_TRUE(io->trigger_user_button_calls[0]);
}

TEST_F(EmergencyStopTest, ResetEStopRefusesWhileVelocityCommandsAreNotZero)
{
    makeEStop([]() { return false; });

    EXPECT_THROW(e_stop->resetEStop(), std::runtime_error);
    // Must not have touched the IO port at all - the invariant is checked before any I/O.
    EXPECT_TRUE(io->trigger_user_button_calls.empty());
}

TEST_F(EmergencyStopTest, ResetEStopSucceedsWhenVelocityCommandsAreZero)
{
    makeEStop([]() { return true; });

    e_stop->resetEStop();

    ASSERT_EQ(io->trigger_user_button_calls.size(), 1u);
    EXPECT_FALSE(io->trigger_user_button_calls[0]);
}

TEST_F(EmergencyStopTest, ResetEStopWithNoZeroVelocityCheckSkipsTheGuard)
{
    makeEStop(nullptr);

    e_stop->resetEStop();

    ASSERT_EQ(io->trigger_user_button_calls.size(), 1u);
    EXPECT_FALSE(io->trigger_user_button_calls[0]);
}

TEST_F(EmergencyStopTest, ResetEStopLatchResetsThePort)
{
    makeEStop([]() { return true; });

    e_stop->resetEStopLatch();

    EXPECT_EQ(io->reset_latch_calls, 1u);
}

}  // namespace rover_hardware_interface
