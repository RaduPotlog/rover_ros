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
// Isolation tests for the two port adapters that sit between the domain and the Modbus-backed
// RoverSafetyController: RoverSafetyControllerEStopIo (EmergencyStopIoPort) and
// RoverSafetyControllerGpioAdapter (RoverGpioPort).
//
// Neither had a test of its own. They are thin, but "thin" is exactly the problem: each is a
// hand-written mapping from a named domain concept to one GPIO enumerator, and getting one of
// those mappings wrong - isLatchActive() reading the contactor pin, say - would be invisible in
// every other test in this package while silently changing what the E-Stop gate believes.

#include <gtest/gtest.h>

#include <memory>

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller.hpp"
#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_e_stop_io.hpp"
#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_gpio_adapter.hpp"

#include "fake_rover_modbus.hpp"

namespace rover_hardware_interface
{
namespace test
{

namespace
{

// Reads of coils and contacts return a single canned value, so a test that wants two pins to
// disagree uses setCoilReadValueFor(). CONTACT_0 is the only contact, so the contact value is
// unambiguous.
class AdapterFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        modbus = std::make_shared<FakeRoverModbus>();
        controller = std::make_shared<RoverSafetyController>(modbus);
        controller->start();
    }

    void TearDown() override
    {
        // Joins the background threads before the fake goes away.
        controller.reset();
    }

    // Waits for the poll thread to have published at least one full sweep, so the adapters read
    // observed values rather than an empty cache.
    bool waitForFirstPoll()
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

        while (std::chrono::steady_clock::now() < deadline) {
            if (!controller->queryControlInterfaceIOStates().empty()) {
                return true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        return !controller->queryControlInterfaceIOStates().empty();
    }

    std::shared_ptr<FakeRoverModbus> modbus;
    std::shared_ptr<RoverSafetyController> controller;
};

}  // namespace

TEST_F(AdapterFixture, EStopIoReadsTheUserButtonFromItsOwnCoil)
{
    modbus->setCoilReadValueFor(Coil::COIL_2, 1);   // GPIO_SW_E_STOP_USER_BUTTON
    modbus->setCoilReadValueFor(Coil::COIL_5, 0);   // GPIO_SW_E_STOP_LATCH_STATUS
    ASSERT_TRUE(waitForFirstPoll());

    RoverSafetyControllerEStopIo io(controller);

    EXPECT_TRUE(io.isUserButtonActive());
    EXPECT_FALSE(io.isLatchActive());
}

TEST_F(AdapterFixture, EStopIoReadsTheLatchFromItsOwnCoil)
{
    modbus->setCoilReadValueFor(Coil::COIL_2, 0);
    modbus->setCoilReadValueFor(Coil::COIL_5, 1);
    ASSERT_TRUE(waitForFirstPoll());

    RoverSafetyControllerEStopIo io(controller);

    EXPECT_FALSE(io.isUserButtonActive());
    EXPECT_TRUE(io.isLatchActive());
}

// The signal the welded-contactor check depends on. Reading the wrong coil here would make the
// cross-check compare the latch against itself and never fire.
TEST_F(AdapterFixture, EStopIoReadsTheContactorFromItsOwnCoil)
{
    modbus->setCoilReadValueFor(Coil::COIL_0, 1);   // GPIO_MOTOR_CONTACTOR_ENGAGED
    modbus->setCoilReadValueFor(Coil::COIL_5, 0);
    ASSERT_TRUE(waitForFirstPoll());

    RoverSafetyControllerEStopIo io(controller);

    EXPECT_TRUE(io.isContactorEngaged());
    EXPECT_FALSE(io.isLatchActive());
}

TEST_F(AdapterFixture, EStopIoTriggerWritesTheUserButtonCoil)
{
    RoverSafetyControllerEStopIo io(controller);

    io.triggerUserButton(true);

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_2, true}));
}

TEST_F(AdapterFixture, EStopIoResetLatchPulsesTheLatchResetCoil)
{
    RoverSafetyControllerEStopIo io(controller);

    io.resetLatch();

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_4, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_4, false}));
}

TEST_F(AdapterFixture, GpioAdapterTriggersMapToTheirOwnCoils)
{
    RoverSafetyControllerGpioAdapter adapter(controller);

    adapter.eStopUserBtnTrigger(true);
    adapter.eStopMotorDriverFaultTrigger(true);

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_2, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_3, true}));
}

TEST_F(AdapterFixture, GpioAdapterExposesEveryMappedPin)
{
    ASSERT_TRUE(waitForFirstPoll());

    RoverSafetyControllerGpioAdapter adapter(controller);
    const auto & states = adapter.queryControlInterfaceIOStates();

    // One contact plus six coils. A pin missing here would silently default-construct as false in
    // the published safety messages.
    EXPECT_EQ(states.size(), 7u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_CPU_WDG_HEARTBEAT), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_SW_E_STOP_LATCH_RESET), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS), 1u);
}

TEST_F(AdapterFixture, GpioAdapterReportsLinkHealthOnceRunning)
{
    ASSERT_TRUE(waitForFirstPoll());

    RoverSafetyControllerGpioAdapter adapter(controller);
    const auto health = adapter.linkHealth();

    EXPECT_TRUE(health.watchdog_running);
    EXPECT_TRUE(health.poll_running);
    EXPECT_NE(health.last_poll_age_ms, SafetyLinkHealth::kUnknownAgeMs);
}

}  // namespace test
}  // namespace rover_hardware_interface
