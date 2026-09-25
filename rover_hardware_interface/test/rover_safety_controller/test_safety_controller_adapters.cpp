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

#include <algorithm>
#include <chrono>
#include <memory>
#include <ostream>
#include <thread>
#include <vector>

#include "rover_hardware_interface/domain/emergency_stop.hpp"
#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller.hpp"
#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_e_stop_io.hpp"
#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_gpio_adapter.hpp"

#include "fake_rover_modbus.hpp"

namespace rover_hardware_interface
{
namespace test
{

// Makes a coil-sequence mismatch readable ("COIL_2=1") instead of gtest's raw byte dump. Outside
// the anonymous namespace so argument-dependent lookup finds it next to CoilWrite.
void PrintTo(const CoilWrite & write, std::ostream * os)
{
    *os << "COIL_" << static_cast<int>(write.coil) << "=" << write.state;
}

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

TEST_F(AdapterFixture, EStopIoMotorDriverFaultTriggerWritesItsOwnCoil)
{
    RoverSafetyControllerEStopIo io(controller);
    // start()'s initCoils() already wrote COIL_3=1, so only a write after the call counts.
    const auto writes_before = modbus->writesSnapshot().size();

    io.triggerMotorDriverFault(true);

    const auto writes = modbus->writesSnapshot();
    const CoilWrite motor_driver_fault_set{Coil::COIL_3, true};
    EXPECT_NE(
        std::find(writes.begin() + writes_before, writes.end(), motor_driver_fault_set),
        writes.end());
}

// What configure puts on the wire: start()'s initCoils() burst, then the release of both
// software E-Stop inputs. This is the exact sequence the removed RoverGpioPort triggers produced
// (checked against them before they went); the heartbeat (COIL_1) is filtered out because its
// interleaving is time-dependent. The zero-velocity check refuses, which proves the release is
// not subject to resetEStop()'s invariant.
TEST_F(AdapterFixture, StartupTriggerReleaseLeavesTheSameCoilSequenceOnTheWire)
{
    EmergencyStop e(std::make_shared<RoverSafetyControllerEStopIo>(controller), [] { return false; });
    e.releaseStartupTriggers();

    std::vector<CoilWrite> writes;
    for (const auto & write : modbus->writesSnapshot()) {
        if (write.coil != Coil::COIL_1) {
            writes.push_back(write);
        }
    }

    const std::vector<CoilWrite> expected = {
        {Coil::COIL_2, true}, {Coil::COIL_3, true}, {Coil::COIL_4, false},
        {Coil::COIL_8, false}, {Coil::COIL_9, false}, {Coil::COIL_10, false},
        {Coil::COIL_11, false}, {Coil::COIL_12, false}, {Coil::COIL_13, false},
        {Coil::COIL_2, false}, {Coil::COIL_3, false},
    };

    EXPECT_EQ(writes, expected);
}

TEST_F(AdapterFixture, GpioAdapterExposesEveryMappedPin)
{
    ASSERT_TRUE(waitForFirstPoll());

    RoverSafetyControllerGpioAdapter adapter(controller);
    const auto & states = adapter.queryControlInterfaceIOStates();

    // One contact, six safety coils and twelve aux coils. A pin missing here would silently
    // default-construct as false in the published messages.
    EXPECT_EQ(states.size(), 19u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_CPU_WDG_HEARTBEAT), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_SW_E_STOP_LATCH_RESET), 1u);
    EXPECT_EQ(states.count(RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS), 1u);

    for (unsigned i = 0; i < kAuxOutputCount; ++i) {
        EXPECT_EQ(states.count(auxOutputPin(i)), 1u) << "aux output " << i;
    }

    for (unsigned i = 0; i < kAuxInputCount; ++i) {
        EXPECT_EQ(states.count(auxInputPin(i)), 1u) << "aux input " << i;
    }
}

// DIO06 is COIL_14 and DIO00 is COIL_8; the aux pins must read their own coils, not a neighbour
// or a safety coil.
TEST_F(AdapterFixture, GpioAdapterReadsTheAuxPinsFromTheirOwnCoils)
{
    modbus->setCoilReadValueFor(Coil::COIL_14, 1);  // DIO06 -> GPIO_AUX_IN_0
    modbus->setCoilReadValueFor(Coil::COIL_13, 1);  // DIO05 -> GPIO_AUX_OUT_5
    ASSERT_TRUE(waitForFirstPoll());

    RoverSafetyControllerGpioAdapter adapter(controller);
    const auto & states = adapter.queryControlInterfaceIOStates();

    EXPECT_TRUE(states.at(RoverControllerGpio::GPIO_AUX_IN_0));
    EXPECT_FALSE(states.at(RoverControllerGpio::GPIO_AUX_IN_1));
    EXPECT_TRUE(states.at(RoverControllerGpio::GPIO_AUX_OUT_5));
    EXPECT_FALSE(states.at(RoverControllerGpio::GPIO_AUX_OUT_0));
}

// Regressions, both seen on the rover's Portenta PLC IDE:
//  - a read crossing from its Digital Outputs area (0..7) into its Programmable DIO area (8..19)
//    is served from the first area only, so every aux bit came back false;
//  - a read of more than 8 coils gets a reply claiming two data bytes but carrying one, so the
//    aux inputs DIO08..11 came back as random garbage (now a rejected reply).
// With the fake modelling both, safety and aux coils must all read their true values.
TEST(AdapterPlcAreasTest, AuxAndSafetyCoilsBothReadCorrectlyOnThePortenta)
{
    auto modbus = std::make_shared<FakeRoverModbus>();
    modbus->setPortentaReadQuirks();
    modbus->setCoilReadValueFor(Coil::COIL_5, 1);   // latch status, Digital Outputs area
    modbus->setCoilReadValueFor(Coil::COIL_8, 1);   // DIO00 -> GPIO_AUX_OUT_0
    modbus->setCoilReadValueFor(Coil::COIL_14, 1);  // DIO06 -> GPIO_AUX_IN_0

    auto controller = std::make_shared<RoverSafetyController>(modbus);
    controller->start();

    RoverSafetyControllerGpioAdapter adapter(controller);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (adapter.queryControlInterfaceIOStates().empty() &&
           std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    const auto & states = adapter.queryControlInterfaceIOStates();
    ASSERT_FALSE(states.empty());

    EXPECT_TRUE(states.at(RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS));
    EXPECT_TRUE(states.at(RoverControllerGpio::GPIO_AUX_OUT_0));
    EXPECT_TRUE(states.at(RoverControllerGpio::GPIO_AUX_IN_0));
    EXPECT_FALSE(states.at(RoverControllerGpio::GPIO_AUX_OUT_1));
    for (unsigned i = 1; i < kAuxInputCount; ++i) {
        EXPECT_FALSE(states.at(auxInputPin(i))) << "aux input " << i;
    }
    EXPECT_EQ(controller->getHealth().poll_error_count, 0u);

    // And no read ever asked the PLC to cross an area boundary.
    for (const auto & request : modbus->coilReadRequests()) {
        const bool in_digital_outputs = request.first + request.count <= 8;
        const bool in_programmable_dio = request.first >= 8 && request.first + request.count <= 20;
        EXPECT_TRUE(in_digital_outputs || in_programmable_dio)
            << "coil read " << request.first << " x" << request.count << " crosses a PLC area";
        EXPECT_LE(request.count, 8u)
            << "coil read " << request.first << " x" << request.count << " exceeds one reply byte";
    }
}

TEST_F(AdapterFixture, GpioAdapterSetAuxOutputWritesItsOwnCoil)
{
    RoverSafetyControllerGpioAdapter adapter(controller);

    adapter.setAuxOutput(0, true);
    adapter.setAuxOutput(5, true);

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_8, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_13, true}));
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
