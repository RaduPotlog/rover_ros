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

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rover_hardware_interface/application/rover_control_loop_use_case.hpp"

namespace rover_hardware_interface
{

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
        // Not exercised by RoverControlLoopUseCase - it never calls getData() directly.
        throw std::logic_error("FakeRoverDriver::getData() is not used by these tests");
    }

    void sendSpeedCmd(const std::vector<float> & /* speeds */) override {}

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

    bool motor_states_data_timed_out = false;
    bool driver_state_data_timed_out = false;
    bool flag_error = false;
    unsigned attempt_error_flag_reset_calls = 0;
    std::optional<std::string> attempt_error_flag_reset_throw_message;
};

class FakeEmergencyStop : public EmergencyStopInterface
{

public:

    bool readEStopState() override { return user_e_stop_triggered; }
    bool readEStopLatchState() override { return latch_active; }
    void setEStop() override {}
    void resetEStop() override {}
    void resetEStopLatch() override {}

    bool user_e_stop_triggered = false;
    bool latch_active = false;
};

class RoverControlLoopUseCaseTest : public ::testing::Test
{
protected:
    std::shared_ptr<FakeRoverDriver> driver = std::make_shared<FakeRoverDriver>();
    std::shared_ptr<FakeEmergencyStop> e_stop = std::make_shared<FakeEmergencyStop>();
    RoverErrorFilter error_filter{1, 1, 1, 1};
    RoverControlLoopUseCase use_case{driver, e_stop, error_filter};
};

TEST_F(RoverControlLoopUseCaseTest, UpdateMotorsStateTimeoutStatusReflectsDriverTimeout)
{
    EXPECT_FALSE(error_filter.isError(ErrorsFilterIds::READ_MOTOR_STATES));

    driver->motor_states_data_timed_out = true;
    use_case.updateMotorsStateTimeoutStatus();

    EXPECT_TRUE(error_filter.isError(ErrorsFilterIds::READ_MOTOR_STATES));
}

TEST_F(RoverControlLoopUseCaseTest, UpdateDriverStateTimeoutStatusReflectsDriverTimeout)
{
    EXPECT_FALSE(error_filter.isError(ErrorsFilterIds::READ_DRIVER_STATE));

    driver->driver_state_data_timed_out = true;
    use_case.updateDriverStateTimeoutStatus();

    EXPECT_TRUE(error_filter.isError(ErrorsFilterIds::READ_DRIVER_STATE));
}

TEST_F(RoverControlLoopUseCaseTest, UpdateFaultFlagStatusReturnsNulloptWhenNoFlagError)
{
    driver->flag_error = false;

    const auto result = use_case.updateFaultFlagStatus();

    EXPECT_FALSE(result.has_value());
    EXPECT_FALSE(error_filter.isError(ErrorsFilterIds::FAULT_FLAG));
    EXPECT_EQ(driver->attempt_error_flag_reset_calls, 0u);
}

TEST_F(RoverControlLoopUseCaseTest, UpdateFaultFlagStatusLatchesAndAttemptsResetWhenFlagErrorActive)
{
    driver->flag_error = true;

    const auto result = use_case.updateFaultFlagStatus();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->outcome, WriteOperationOutcome::kSucceeded);
    EXPECT_TRUE(error_filter.isError(ErrorsFilterIds::FAULT_FLAG));
    EXPECT_EQ(driver->attempt_error_flag_reset_calls, 1u);
    EXPECT_FALSE(error_filter.isError(ErrorsFilterIds::WRITE_CMDS));
}

TEST_F(RoverControlLoopUseCaseTest, UpdateFaultFlagStatusReportsExceptionFromResetAttempt)
{
    driver->flag_error = true;
    driver->attempt_error_flag_reset_throw_message = "reset failed";

    const auto result = use_case.updateFaultFlagStatus();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->outcome, WriteOperationOutcome::kExceptionThrown);
    EXPECT_EQ(result->error_message, "reset failed");
    EXPECT_TRUE(error_filter.isError(ErrorsFilterIds::WRITE_CMDS));
}

TEST_F(RoverControlLoopUseCaseTest, UpdateEStopActiveStateReflectsUserTrigger)
{
    e_stop->user_e_stop_triggered = true;
    e_stop->latch_active = false;

    EXPECT_TRUE(use_case.updateEStopActiveState());
}

TEST_F(RoverControlLoopUseCaseTest, UpdateEStopActiveStateReflectsLatch)
{
    e_stop->user_e_stop_triggered = false;
    e_stop->latch_active = true;

    EXPECT_TRUE(use_case.updateEStopActiveState());
}

TEST_F(RoverControlLoopUseCaseTest, UpdateEStopActiveStateFalseWhenBothClear)
{
    e_stop->user_e_stop_triggered = false;
    e_stop->latch_active = false;

    EXPECT_FALSE(use_case.updateEStopActiveState());
}

TEST_F(RoverControlLoopUseCaseTest, PerformWriteOperationSucceeds)
{
    bool called = false;
    const auto result = use_case.performWriteOperation([&called] { called = true; });

    EXPECT_TRUE(called);
    EXPECT_EQ(result.outcome, WriteOperationOutcome::kSucceeded);
    EXPECT_FALSE(error_filter.isError(ErrorsFilterIds::WRITE_CMDS));
}

TEST_F(RoverControlLoopUseCaseTest, PerformWriteOperationReportsException)
{
    const auto result = use_case.performWriteOperation([] { throw std::runtime_error("boom"); });

    EXPECT_EQ(result.outcome, WriteOperationOutcome::kExceptionThrown);
    EXPECT_EQ(result.error_message, "boom");
    EXPECT_TRUE(error_filter.isError(ErrorsFilterIds::WRITE_CMDS));
}

TEST_F(RoverControlLoopUseCaseTest, PerformWriteOperationReportsLockContentionWithoutInvokingOperation)
{
    // Hold the internal write lock from another thread long enough to force contention, mirroring
    // how a concurrent caller is observed by performWriteOperation()'s try_lock (see its RT-safety
    // contract in rover_control_loop_use_case.hpp). Sleeping to synchronize is normally avoided
    // (see .claude/rules/testing.md), but the thing under test here IS a raw try_lock with no
    // future/condition to wait on instead - there is no non-sleep way to guarantee the holder
    // thread is inside the critical section before the second performWriteOperation() call.
    std::thread holder([this] {
        use_case.performWriteOperation(
            [] { std::this_thread::sleep_for(std::chrono::milliseconds(200)); });
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    bool called = false;
    const auto result = use_case.performWriteOperation([&called] { called = true; });

    EXPECT_FALSE(called);
    EXPECT_EQ(result.outcome, WriteOperationOutcome::kLockContention);
    EXPECT_TRUE(error_filter.isError(ErrorsFilterIds::WRITE_CMDS));

    holder.join();
}

TEST(RoverControlLoopUseCaseShouldCommandMotionTest, TrueOnlyWhenActiveAndEStopNotActive)
{
    EXPECT_TRUE(RoverControlLoopUseCase::shouldCommandMotion(true, false));
    EXPECT_FALSE(RoverControlLoopUseCase::shouldCommandMotion(true, true));
    EXPECT_FALSE(RoverControlLoopUseCase::shouldCommandMotion(false, false));
    EXPECT_FALSE(RoverControlLoopUseCase::shouldCommandMotion(false, true));
}

TEST(RoverControlLoopUseCaseNoEStopTest, UpdateEStopActiveStateFailsSafeWithNoEStopConfigured)
{
    auto driver = std::make_shared<FakeRoverDriver>();
    RoverErrorFilter error_filter{1, 1, 1, 1};
    RoverControlLoopUseCase use_case(driver, nullptr, error_filter);

    EXPECT_TRUE(use_case.updateEStopActiveState());
}

}  // namespace rover_hardware_interface
