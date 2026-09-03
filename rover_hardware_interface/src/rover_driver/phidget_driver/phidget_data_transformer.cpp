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

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_data_transformer.hpp"

#include <cmath>

namespace rover_hardware_interface
{

PhidgetVelocityCommandDataTransformer::PhidgetVelocityCommandDataTransformer(
    const DrivetrainSettings & drivetrain_settings)
{
    radians_per_second_to_phidget_cmd_ = drivetrain_settings.gear_ratio * (1.0f / (2.0f * M_PI)) *
                                         60.0f * (kMaxPhidgetCmdValue / drivetrain_settings.max_rpm_motor_speed);
}

float PhidgetVelocityCommandDataTransformer::convert(const float cmd) const
{
    return clampVelCmd(cmd * radians_per_second_to_phidget_cmd_);
}

}  // namespace rover_hardware_interface
