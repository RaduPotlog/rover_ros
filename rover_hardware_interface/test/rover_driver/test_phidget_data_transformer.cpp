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

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_data_transformer.hpp"

namespace rover_hardware_interface
{

namespace
{

DrivetrainSettings makeSettings(const float gear_ratio, const float max_rpm_motor_speed)
{
    DrivetrainSettings settings;
    settings.motor_torque_constant = 1.0f;
    settings.gear_ratio = gear_ratio;
    settings.gearbox_efficiency = 1.0f;
    settings.encoder_resolution = 1.0f;
    settings.max_rpm_motor_speed = max_rpm_motor_speed;
    settings.driver_comm_timeout_ms = 100u;
    return settings;
}

}  // namespace

// gear_ratio and max_rpm_motor_speed are chosen so that
// gear_ratio * (1 / (2*pi)) * 60 * (1 / max_rpm_motor_speed) == 1.0,
// i.e. the transformer becomes an identity map (before clamping) - this isolates the test from
// having to duplicate the conversion formula to compute an expected value.
TEST(PhidgetVelocityCommandDataTransformerTest, ConvertsWithinRangeUnchanged)
{
    const auto settings = makeSettings(1.0f, 60.0f / (2.0f * static_cast<float>(M_PI)));
    PhidgetVelocityCommandDataTransformer transformer(settings);

    EXPECT_NEAR(transformer.convert(0.5f), 0.5f, 1e-4f);
    EXPECT_NEAR(transformer.convert(-0.5f), -0.5f, 1e-4f);
    EXPECT_NEAR(transformer.convert(0.0f), 0.0f, 1e-4f);
}

TEST(PhidgetVelocityCommandDataTransformerTest, ClampsAboveUpperBound)
{
    const auto settings = makeSettings(1.0f, 60.0f / (2.0f * static_cast<float>(M_PI)));
    PhidgetVelocityCommandDataTransformer transformer(settings);

    EXPECT_FLOAT_EQ(transformer.convert(2.0f), 1.0f);
}

TEST(PhidgetVelocityCommandDataTransformerTest, ClampsBelowLowerBound)
{
    const auto settings = makeSettings(1.0f, 60.0f / (2.0f * static_cast<float>(M_PI)));
    PhidgetVelocityCommandDataTransformer transformer(settings);

    EXPECT_FLOAT_EQ(transformer.convert(-2.0f), -1.0f);
}

TEST(PhidgetVelocityCommandDataTransformerTest, ScalesWithGearRatio)
{
    // Doubling gear_ratio doubles the phidget command magnitude for the same input speed.
    const auto settings = makeSettings(2.0f, 60.0f / (2.0f * static_cast<float>(M_PI)));
    PhidgetVelocityCommandDataTransformer transformer(settings);

    EXPECT_NEAR(transformer.convert(0.25f), 0.5f, 1e-4f);
}

}  // namespace rover_hardware_interface
