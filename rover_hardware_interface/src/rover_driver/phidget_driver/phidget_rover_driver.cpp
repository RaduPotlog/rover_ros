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

#include <chrono>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_rover_driver.hpp"

namespace rover_hardware_interface {

PhidgetRoverDriver::PhidgetRoverDriver(
    const DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time)
: drivetrain_settings_(drivetrain_settings)
, phidget_vel_cmd_converter_(drivetrain_settings)
, activate_wait_time_(activate_wait_time)
{

}

PhidgetRoverDriver::~PhidgetRoverDriver()
{
    drivers_.clear();
    deinitialize();
}

void PhidgetRoverDriver::initialize()
{
    if (initialized_) {
        return;
    }

    try {
        defineDrivers();
        
        for (auto & [name, driver] : drivers_) {
            data_.emplace(name, PhidgetDriverDataTransformer(drivetrain_settings_));
        }

        initDrivers();
        
    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Failed to initialize robot driver: " + std::string(e.what()));
    }

    initialized_ = true;
}

void PhidgetRoverDriver::deinitialize()
{
    drivers_.clear();
    initialized_ = false;
}

void PhidgetRoverDriver::activate()
{
    for (auto & [name, driver] : drivers_) {
        try {
            driver->getMotorDriver(MotorNames::DEFAULT)->sendCmdVel(0);
        } catch (const std::runtime_error & e) {
            throw std::runtime_error(
                "Send command exception on " + driverNamesToString(name) + 
                "driver : " + std::string(e.what()));
        }
    }

    std::this_thread::sleep_for(activate_wait_time_);
}

void PhidgetRoverDriver::updateCommunicationStatus()
{
    // TODO: Handle comm error
}

void PhidgetRoverDriver::updateMotorsState()
{
    timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    for (auto & [name, driver] : drivers_) {
        const auto state = driver->getMotorDriver(MotorNames::DEFAULT)->readState();
        setMotorsStates(data_.at(name), state, current_time);
    }
}

void PhidgetRoverDriver::updateDriversState()
{
    timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    for (auto & [name, driver] : drivers_) {
        setDriverState(data_.at(name), driver->readState(), current_time);
    }
}

void PhidgetRoverDriver::attemptErrorFlagReset()
{

}

bool PhidgetRoverDriver::isCommunicationError()
{  
    return false;
}

const PhidgetDriverDataTransformer & PhidgetRoverDriver::getData(const DriverNames name)
{
    if (data_.find(name) == data_.end()) {
        throw std::runtime_error("Data with name '" + driverNamesToString(name) + "' does not exist.");
    }

    return data_.at(name);
}

PhidgetVelocityCommandDataTransformer & PhidgetRoverDriver::getCmdVelConverter() 
{ 
    return phidget_vel_cmd_converter_; 
}

void PhidgetRoverDriver::initDrivers()
{
    for (auto & [name, driver] : drivers_) {
        const auto name_str = driverNamesToString(name);
        
        try {
            auto driver_future = driver->initialize();
            auto driver_status = driver_future.wait_for(std::chrono::seconds(1));

            if (driver_status == std::future_status::ready) {
                try {
                    driver_future.get();
                } catch (const std::exception & e) {
                    throw std::runtime_error(
                        "Init driver for " + name_str + " driver failed with exception: " + std::string(e.what()));
                }
            } else {
                throw std::runtime_error("Init driver for " + name_str + " driver timed out or failed.");
            }
        } catch (const std::runtime_error & e) {
            throw std::runtime_error("Send command exception on " + driverNamesToString(name) + " driver: " + std::string(e.what()));
        }
    }
}

void PhidgetRoverDriver::setMotorsStates(
    PhidgetDriverDataTransformer & data, 
    const MotorDriverState & state,
    const timespec & current_time)
{
    const bool data_timed_out = false;

    // TODO: Handle timeout
    (void)current_time;

    data.setMotorsStates(state, data_timed_out);
}
    
void PhidgetRoverDriver::setDriverState(
    PhidgetDriverDataTransformer & data, 
    const DriverState & state, 
    const timespec & current_time)
{
    const bool data_timed_out = false;

    // TODO: Handle timeout
    (void)current_time;

    data.setDriverState(state, data_timed_out);
}

} // namespace rover_hardware_interface