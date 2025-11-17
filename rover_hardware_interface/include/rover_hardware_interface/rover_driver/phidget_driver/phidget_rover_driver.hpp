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

#ifndef ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_ROVER_DRIVER_PHIDGET_ROVER_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_ROVER_DRIVER_PHIDGET_ROVER_DRIVER_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "rover_hardware_interface/rover_driver/driver.hpp"
#include "rover_hardware_interface/rover_driver/rover_driver.hpp"
#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"
#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_data_transformer.hpp"

namespace rover_hardware_interface
{

struct MotorChannels
{
    static constexpr std::uint8_t DEFAULT = PhidgetDriver::motorChannelDefault;
};

class PhidgetRoverDriver : public RoverDriverInterface
{

public:

    PhidgetRoverDriver(
        const DrivetrainSettings & drivetrain_settings,
        const std::chrono::milliseconds activate_wait_time = std::chrono::milliseconds(1000));

    ~PhidgetRoverDriver();

    void initialize() override;

    void deinitialize() override;

    void activate() override;

    void updateCommunicationStatus() override;

    void updateMotorsState() override;

    void updateDriversState() override;

    void attemptErrorFlagReset() override;

    bool isCommunicationError() override;

    const PhidgetDriverDataTransformer & getData(const DriverNames name) override;

protected:

    virtual void defineDrivers() = 0;

    PhidgetVelocityCommandDataTransformer & getCmdVelConverter();

    // Drive train system settings
    DrivetrainSettings drivetrain_settings_;

    std::unordered_map<DriverNames, std::shared_ptr<DriverInterface>> drivers_;
    
private:

    void initDrivers();

    void setMotorsStates(
        PhidgetDriverDataTransformer & data, 
        const MotorDriverState & state,
        const timespec & current_time);
    
    void setDriverState(
        PhidgetDriverDataTransformer & data, 
        const DriverState & state, 
        const timespec & current_time);

    bool initialized_ = false;

    std::unordered_map<DriverNames, PhidgetDriverDataTransformer> data_;

    PhidgetVelocityCommandDataTransformer phidget_vel_cmd_converter_;

    const std::chrono::milliseconds activate_wait_time_;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_ROVER_DRIVER_PHIDGET_ROVER_DRIVER_HPP_