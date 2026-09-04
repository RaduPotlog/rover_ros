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
// Unit tests for RoverSafetyController's enabled-guard and delegation logic, run against a
// FakeRoverModbus via the test-only injecting constructor (see rover_safety_controller.hpp) - no
// real Modbus TCP connection involved. Assertions avoid depending on ContactCoilHandler's
// background poll thread completing an iteration (see test_contact_coil_handler.cpp's file
// comment for why).

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller.hpp"

#include "fake_rover_modbus.hpp"

namespace rover_hardware_interface
{
namespace test
{

namespace
{

// ContactCoilHandler's poll thread populates its IO-state cache asynchronously with no other
// signaling primitive exposed (see ContactCoilHandler::contactCoilHandlerThread()) - polls with a
// bounded timeout rather than sleeping a fixed guessed duration, so the test resolves as soon as
// the state is ready and still fails deterministically (not hangs) if it never is.
template <typename Predicate>
bool waitUntil(Predicate predicate, const std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return predicate();
}

}  // namespace

class RoverSafetyControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        modbus = std::make_shared<FakeRoverModbus>();
        controller = std::make_unique<RoverSafetyController>(modbus);
    }

    std::shared_ptr<FakeRoverModbus> modbus;
    std::unique_ptr<RoverSafetyController> controller;
};

// Regression test for a null-pointer-dereference hazard: contactCoilHandler_ is null until
// start() is called, and the guarded methods below used to dereference it before checking that
// (see the fix in rover_safety_controller.cpp) - calling any of them before start() must be a
// safe no-op instead of a crash.
TEST_F(RoverSafetyControllerTest, GuardedMethodsAreSafeNoOpsBeforeStart)
{
    controller->eStopUserBtnTrigger(true);
    controller->eStopMotorDriverFaultTrigger(true);
    controller->eStopLatchReset();
    const auto & io_state = controller->queryControlInterfaceIOStates();

    EXPECT_TRUE(io_state.empty());
    EXPECT_TRUE(modbus->writesSnapshot().empty());
    EXPECT_FALSE(controller->isPinActive(RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED));
}

TEST_F(RoverSafetyControllerTest, EStopUserBtnTriggerDelegatesAfterStart)
{
    controller->start();

    controller->eStopUserBtnTrigger(true);

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_2, true}));
}

TEST_F(RoverSafetyControllerTest, EStopMotorDriverFaultTriggerDelegatesAfterStart)
{
    controller->start();

    controller->eStopMotorDriverFaultTrigger(true);

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_3, true}));
}

TEST_F(RoverSafetyControllerTest, EStopLatchResetDelegatesAfterStart)
{
    controller->start();

    controller->eStopLatchReset();

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_4, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_4, false}));
}

TEST_F(RoverSafetyControllerTest, IsPinActiveReflectsConfiguredContactReadValue)
{
    modbus->setContactReadValue(1);
    controller->start();

    const bool became_active = waitUntil(
        [this]() {
            return controller->isPinActive(RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN);
        },
        std::chrono::milliseconds(1000));

    EXPECT_TRUE(became_active);
}

}  // namespace test
}  // namespace rover_hardware_interface
