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

#include "rover_hardware_interface/rover_driver/rover_a1_driver.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"
#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_rover_driver.hpp"

namespace rover_hardware_interface
{

RoverA1Driver::RoverA1Driver(
    const DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time)
: PhidgetRoverDriver(drivetrain_settings, activate_wait_time)
{

}

void RoverA1Driver::sendSpeedCmd(const std::vector<float> & speeds)
{
    if (speeds.size() != 4) {
        throw std::runtime_error(
            "Invalid speeds vector size. Expected 4, got " + 
            std::to_string(speeds.size()));
    }

    const auto speed_rear_left = this->getCmdVelConverter().convert(speeds.at(0));
    const auto speed_rear_right = this->getCmdVelConverter().convert(speeds.at(1));
    const auto speed_front_left = this->getCmdVelConverter().convert(speeds.at(2));
    const auto speed_front_right = this->getCmdVelConverter().convert(speeds.at(3));

    try {
        drivers_.at(DriverNames::REAR_LEFT)->getMotorDriver(MotorNames::DEFAULT)->sendCmdVel(speed_rear_left);
        drivers_.at(DriverNames::REAR_RIGHT)->getMotorDriver(MotorNames::DEFAULT)->sendCmdVel(speed_rear_right);
        drivers_.at(DriverNames::FRONT_LEFT)->getMotorDriver(MotorNames::DEFAULT)->sendCmdVel(speed_front_left);
        drivers_.at(DriverNames::FRONT_RIGHT)->getMotorDriver(MotorNames::DEFAULT)->sendCmdVel(speed_front_right);
    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Driver send cmd failed: " + std::string(e.what()));
    }
}

void RoverA1Driver::attemptErrorFlagReset()
{ 
    sendSpeedCmd({0.0, 0.0});
}

void RoverA1Driver::defineDrivers()
{  
    auto rear_left_driver = std::make_shared<PhidgetDriver>();
    auto rear_right_driver = std::make_shared<PhidgetDriver>();
    auto front_left_driver = std::make_shared<PhidgetDriver>();
    auto front_right_driver = std::make_shared<PhidgetDriver>();

    auto rear_left_motor_driver = std::make_shared<PhidgetMotorDriver>(
        std::dynamic_pointer_cast<PhidgetDriver>(rear_left_driver), 0, -1, true);
    auto rear_right_motor_driver = std::make_shared<PhidgetMotorDriver>(
        std::dynamic_pointer_cast<PhidgetDriver>(rear_right_driver), 5, -1, false);
    auto front_left_motor_driver = std::make_shared<PhidgetMotorDriver>(
        std::dynamic_pointer_cast<PhidgetDriver>(front_left_driver), 1, -1, true);
    auto front_right_motor_driver = std::make_shared<PhidgetMotorDriver>(
        std::dynamic_pointer_cast<PhidgetDriver>(front_right_driver), 4, -1, false);
    
    rear_left_driver->addMotorDriver(MotorNames::DEFAULT, rear_left_motor_driver);
    rear_right_driver->addMotorDriver(MotorNames::DEFAULT, rear_right_motor_driver);
    front_left_driver->addMotorDriver(MotorNames::DEFAULT, front_left_motor_driver);
    front_right_driver->addMotorDriver(MotorNames::DEFAULT, front_right_motor_driver);

    drivers_.emplace(DriverNames::REAR_LEFT, rear_left_driver);
    drivers_.emplace(DriverNames::REAR_RIGHT, rear_right_driver);
    drivers_.emplace(DriverNames::FRONT_LEFT, front_left_driver);
    drivers_.emplace(DriverNames::FRONT_RIGHT, front_right_driver);
}

}  // namespace rover_hardware_interface