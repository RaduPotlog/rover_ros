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

#ifndef ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_ROVER_A1_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_ROVER_A1_DRIVER_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_rover_driver.hpp"

namespace rover_hardware_interface
{

class RoverA1Driver : public PhidgetRoverDriver
{

public:

    RoverA1Driver(
        const DrivetrainSettings & drivetrain_settings,
        const std::chrono::milliseconds activate_wait_time = std::chrono::milliseconds(1000));

    ~RoverA1Driver() = default;

    void sendSpeedCmd(const std::vector<float> & speeds);

    void attemptErrorFlagReset();

protected:
  
    void defineDrivers() override;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_ROVER_A1_DRIVER_HPP_
