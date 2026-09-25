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

#ifndef ROVER_HARDWARE_INTERFACE_TEST_FAKES_FAKE_ROVER_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACE_TEST_FAKES_FAKE_ROVER_DRIVER_HPP_

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rover_hardware_interface/domain/emergency_stop.hpp"
#include "rover_hardware_interface/domain/rover_driver.hpp"

namespace rover_hardware_interface
{

// In-memory RoverDriverInterface: no Phidget hardware. Records every sendSpeedCmd() payload and
// exposes the error/timeout/failsafe flags as plain members so tests can drive them directly.
// Shared by test_rover_control_loop_use_case and test_rover_system_write.
class FakeRoverDriver : public RoverDriverInterface
{

public:

    void initialize() override {}
    void deinitialize() override {}
    void activate() override {}
    void updateCommunicationStatus() override {}
    void updateMotorsState() override {}
    void updateDriversState() override {}

    DriverDataSnapshot getData(const DriverNames /* name */) override
    {
        // Not exercised by the tests using this fake - none of them call getData().
        throw std::logic_error("FakeRoverDriver::getData() is not used by these tests");
    }

    void sendSpeedCmd(const std::vector<float> & speeds) override
    {
        sent_speed_cmds.push_back(speeds);
    }

    void attemptErrorFlagReset() override
    {
        ++attempt_error_flag_reset_calls;

        if (attempt_error_flag_reset_throw_message) {
            throw std::runtime_error(*attempt_error_flag_reset_throw_message);
        }
    }

    bool isCommunicationError() override { return false; }

    bool isMotorStatesDataTimedOut() override { return motor_states_data_timed_out; }
    bool isDriverStateDataTimedOut() override { return driver_state_data_timed_out; }
    bool isFlagError() override { return flag_error; }

    void armFailsafe() override { ++arm_failsafe_calls; }
    void resetFailsafe() override { ++reset_failsafe_calls; }
    bool isFailsafeTripped() override { return failsafe_tripped; }

    bool motor_states_data_timed_out = false;
    bool driver_state_data_timed_out = false;
    bool flag_error = false;
    unsigned attempt_error_flag_reset_calls = 0;
    std::optional<std::string> attempt_error_flag_reset_throw_message;
    bool failsafe_tripped = false;
    unsigned arm_failsafe_calls = 0;
    unsigned reset_failsafe_calls = 0;
    std::vector<std::vector<float>> sent_speed_cmds;
};

class FakeEmergencyStop : public EmergencyStopInterface
{

public:

    bool readEStopState() override { return user_e_stop_triggered; }
    bool readEStopLatchState() override { return latch_active; }

    bool readContactorEngagedState() override { return contactor_engaged; }
    void setEStop() override {}
    void resetEStop() override {}
    void resetEStopLatch() override {}
    void releaseStartupTriggers() override { ++release_startup_triggers_calls; }

    bool user_e_stop_triggered = false;
    bool latch_active = false;
    bool contactor_engaged = true;
    unsigned release_startup_triggers_calls = 0;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_TEST_FAKES_FAKE_ROVER_DRIVER_HPP_
