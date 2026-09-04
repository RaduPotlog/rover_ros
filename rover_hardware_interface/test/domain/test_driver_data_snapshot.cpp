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

#include <cmath>

#include "rover_hardware_interface/domain/driver_data_snapshot.hpp"

namespace rover_hardware_interface
{

TEST(FaultFlagTest, StartsWithNoError)
{
    FaultFlag fault_flag;

    EXPECT_FALSE(fault_flag.isError());
    EXPECT_FALSE(fault_flag.isEmergencyStop());
    EXPECT_FALSE(fault_flag.isMotorSetupFault());
}

TEST(FaultFlagTest, DecodesEmergencyStopBit)
{
    FaultFlag fault_flag;
    fault_flag.setData(0b01U);

    EXPECT_TRUE(fault_flag.isError());
    EXPECT_TRUE(fault_flag.isEmergencyStop());
    EXPECT_FALSE(fault_flag.isMotorSetupFault());
}

TEST(FaultFlagTest, DecodesMotorSetupFaultBit)
{
    FaultFlag fault_flag;
    fault_flag.setData(0b10U);

    EXPECT_TRUE(fault_flag.isError());
    EXPECT_FALSE(fault_flag.isEmergencyStop());
    EXPECT_TRUE(fault_flag.isMotorSetupFault());
}

TEST(FaultFlagTest, GetErrorMapUsesPrefixedNamesAndReflectsState)
{
    FaultFlag fault_flag;
    fault_flag.setData(0b01U);

    const auto error_map = fault_flag.getErrorMap();

    EXPECT_TRUE(error_map.at("fault_flag.emergency_stop"));
    EXPECT_FALSE(error_map.at("fault_flag.motor_setup_fault"));
}

// RuntimeError suppresses its only flag (safety_stop_active) from isError() by design (see
// RuntimeError's constructor in driver_data_snapshot.cpp): it is reported informationally via
// isSafetyStopActive(), not treated as an isError()-worthy fault on its own.
TEST(RuntimeErrorTest, SafetyStopActiveIsSuppressedFromIsErrorButNotFromItsOwnAccessor)
{
    RuntimeError runtime_error;
    runtime_error.setData(0b1U);

    EXPECT_TRUE(runtime_error.isSafetyStopActive());
    EXPECT_FALSE(runtime_error.isError());
}

namespace
{

DrivetrainSettings makeDrivetrainSettings()
{
    DrivetrainSettings settings;
    settings.motor_torque_constant = 1.0f;
    settings.gear_ratio = 1.0f;
    settings.gearbox_efficiency = 1.0f;
    settings.encoder_resolution = 4.0f;  // 4 ticks/rev before the /4.0 quadrature divide below
    settings.max_rpm_motor_speed = 100.0f;
    settings.driver_comm_timeout_ms = 500;
    settings.raw_current_to_amps_scale = 1.0f;
    return settings;
}

}  // namespace

class DriverDataSnapshotTest : public ::testing::Test
{
protected:
    DriverDataSnapshotTest() : snapshot_(makeDrivetrainSettings()) {}

    DriverDataSnapshot snapshot_;
};

TEST_F(DriverDataSnapshotTest, StartsWithoutErrorOrTimeout)
{
    EXPECT_FALSE(snapshot_.isError());
    EXPECT_FALSE(snapshot_.isFlagError());
    EXPECT_FALSE(snapshot_.isCommunicationError());
    EXPECT_FALSE(snapshot_.isMotorStatesDataTimedOut());
    EXPECT_FALSE(snapshot_.isDriverStateDataTimedOut());
}

TEST_F(DriverDataSnapshotTest, MotorStatesTimeoutMarksCommunicationAndOverallError)
{
    snapshot_.setMotorsStates(MotorDriverState{0, 0, 0, 0.0f}, /*data_timed_out=*/true);

    EXPECT_TRUE(snapshot_.isMotorStatesDataTimedOut());
    EXPECT_TRUE(snapshot_.isCommunicationError());
    EXPECT_TRUE(snapshot_.isError());
    // A motor-state timeout is a comms problem, not a fault-flag problem.
    EXPECT_FALSE(snapshot_.isFlagError());
}

TEST_F(DriverDataSnapshotTest, DriverStateFaultFlagsMarkFlagAndOverallErrorButNotCommunication)
{
    DriverState state{};
    state.fault_flags = 0b01U;  // emergency_stop

    snapshot_.setDriverState(state, /*data_timed_out=*/false);

    EXPECT_TRUE(snapshot_.isFlagError());
    EXPECT_TRUE(snapshot_.isError());
    EXPECT_FALSE(snapshot_.isCommunicationError());
    EXPECT_TRUE(snapshot_.getFaultFlag().isEmergencyStop());
}

TEST_F(DriverDataSnapshotTest, GetMotorStateConvertsRawEncoderTicksUsingDrivetrainSettings)
{
    // pos=4 ticks with encoder_resolution=4 -> 1 full revolution -> 2*pi radians.
    snapshot_.setMotorsStates(MotorDriverState{4, 0, 0, 0.0f}, false);

    const auto & motor_state = snapshot_.getMotorState(MotorNames::DEFAULT);

    EXPECT_NEAR(motor_state.getPosition(), 2.0 * M_PI, 1e-4);
}

TEST_F(DriverDataSnapshotTest, GetErrorMapMergesFlagAndTimeoutState)
{
    DriverState state{};
    state.fault_flags = 0b01U;
    snapshot_.setDriverState(state, /*data_timed_out=*/true);

    const auto error_map = snapshot_.getErrorMap();

    EXPECT_TRUE(error_map.at("fault_flag.emergency_stop"));
    EXPECT_TRUE(error_map.at("driver_state_data_timed_out"));
    EXPECT_FALSE(error_map.at("motor_states_data_timed_out"));
}

}  // namespace rover_hardware_interface
