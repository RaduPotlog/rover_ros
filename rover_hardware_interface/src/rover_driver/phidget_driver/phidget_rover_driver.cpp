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
            data_.emplace(name, DriverDataSnapshot(drivetrain_settings_));
            last_known_data_.emplace(name, data_.at(name));
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
    bool has_comm_error = false;

    for (auto & [name, driver] : drivers_) {
        if (driver->isCommunicationError()) {
            has_comm_error = true;
            break;
        }
    }

    has_communication_error_ = has_comm_error;
}

void PhidgetRoverDriver::updateMotorsState()
{
    for (auto & [name, driver] : drivers_) {
        auto motor_driver = driver->getMotorDriver(MotorNames::DEFAULT);
        const auto state = motor_driver->readState();
        const bool data_timed_out = motor_driver->isCommunicationError();

        std::lock_guard<std::mutex> lck(data_mtx_);
        setMotorsStates(data_.at(name), state, data_timed_out);
    }
}

void PhidgetRoverDriver::updateDriversState()
{
    for (auto & [name, driver] : drivers_) {
        const auto state = driver->readState();
        const bool data_timed_out = driver->isCommunicationError();

        std::lock_guard<std::mutex> lck(data_mtx_);
        setDriverState(data_.at(name), state, data_timed_out);
    }
}

void PhidgetRoverDriver::attemptErrorFlagReset()
{

}

bool PhidgetRoverDriver::isCommunicationError()
{
    return has_communication_error_;
}

bool PhidgetRoverDriver::isMotorStatesDataTimedOut()
{
    for (const auto & [name, driver_data] : data_) {
        if (driver_data.isMotorStatesDataTimedOut()) {
            return true;
        }
    }

    return false;
}

bool PhidgetRoverDriver::isDriverStateDataTimedOut()
{
    for (const auto & [name, driver_data] : data_) {
        if (driver_data.isDriverStateDataTimedOut()) {
            return true;
        }
    }

    return false;
}

bool PhidgetRoverDriver::isFlagError()
{
    for (const auto & [name, driver_data] : data_) {
        if (driver_data.isFlagError()) {
            return true;
        }
    }

    return false;
}

DriverDataSnapshot PhidgetRoverDriver::getData(const DriverNames name)
{
    // `last_known_data_` mirrors `data_`'s keys 1:1 (both seeded together in initialize(), never
    // modified structurally afterwards), so this check doesn't need data_mtx_.
    if (last_known_data_.find(name) == last_known_data_.end()) {
        throw std::runtime_error("Data with name '" + driverNamesToString(name) + "' does not exist.");
    }

    // Non-blocking: this may run on the RT thread (via RoverA1System::updateHwStates()), which
    // must never block waiting on the diagnostics thread's own getData() call. On contention,
    // return the last successfully-read snapshot instead - see last_known_data_'s comment.
    std::unique_lock<std::mutex> lck(data_mtx_, std::try_to_lock);
    if (lck.owns_lock()) {
        last_known_data_.at(name) = data_.at(name);
    }

    return last_known_data_.at(name);
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
    DriverDataSnapshot & data,
    const MotorDriverState & state,
    const bool data_timed_out)
{
    data.setMotorsStates(state, data_timed_out);
}

void PhidgetRoverDriver::setDriverState(
    DriverDataSnapshot & data,
    const DriverState & state,
    const bool data_timed_out)
{
    data.setDriverState(state, data_timed_out);
}

} // namespace rover_hardware_interface