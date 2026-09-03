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

#include <limits>

#include "rover_hardware_interface/domain/imu_calibration.hpp"

namespace rover_hardware_interface
{

TEST(IsVectorFiniteTest, TrueForAllFiniteComponents)
{
    EXPECT_TRUE(isVectorFinite(Vector3{1.0, -2.0, 3.5}));
}

TEST(IsVectorFiniteTest, FalseIfAnyComponentIsNan)
{
    const auto nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(isVectorFinite(Vector3{nan, 0.0, 0.0}));
    EXPECT_FALSE(isVectorFinite(Vector3{0.0, nan, 0.0}));
    EXPECT_FALSE(isVectorFinite(Vector3{0.0, 0.0, nan}));
}

TEST(IsVectorFiniteTest, FalseIfAnyComponentIsInfinite)
{
    const auto inf = std::numeric_limits<double>::infinity();

    EXPECT_FALSE(isVectorFinite(Vector3{inf, 0.0, 0.0}));
}

TEST(IsMagnitudeSynchronizedTest, DelegatesToIsVectorFinite)
{
    const auto nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_TRUE(isMagnitudeSynchronizedWithAccelerationAndGyration(Vector3{1.0, 1.0, 1.0}));
    EXPECT_FALSE(isMagnitudeSynchronizedWithAccelerationAndGyration(Vector3{nan, 1.0, 1.0}));
}

TEST(ImuCalibrationGateTest, StartsUncalibrated)
{
    ImuCalibrationGate gate;
    EXPECT_FALSE(gate.isCalibrated());
}

TEST(ImuCalibrationGateTest, NonFiniteSampleDoesNotCalibrate)
{
    ImuCalibrationGate gate;
    const auto nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(gate.update(Vector3{nan, nan, nan}));
    EXPECT_FALSE(gate.isCalibrated());
}

TEST(ImuCalibrationGateTest, FirstFiniteSampleLatchesCalibrated)
{
    ImuCalibrationGate gate;

    EXPECT_TRUE(gate.update(Vector3{1.0, 2.0, 3.0}));
    EXPECT_TRUE(gate.isCalibrated());
}

TEST(ImuCalibrationGateTest, StaysCalibratedEvenIfLaterSampleIsNonFinite)
{
    ImuCalibrationGate gate;
    const auto nan = std::numeric_limits<double>::quiet_NaN();

    ASSERT_TRUE(gate.update(Vector3{1.0, 2.0, 3.0}));

    EXPECT_TRUE(gate.update(Vector3{nan, nan, nan}));
    EXPECT_TRUE(gate.isCalibrated());
}

}  // namespace rover_hardware_interface
