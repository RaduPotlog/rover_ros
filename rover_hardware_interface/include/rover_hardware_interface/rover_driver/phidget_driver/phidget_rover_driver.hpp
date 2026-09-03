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

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rover_hardware_interface/domain/driver.hpp"
#include "rover_hardware_interface/domain/rover_driver.hpp"
#include "rover_hardware_interface/domain/driver_data_snapshot.hpp"
#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"
#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_data_transformer.hpp"

namespace rover_hardware_interface
{

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

    bool isMotorStatesDataTimedOut() override;

    bool isDriverStateDataTimedOut() override;

    bool isFlagError() override;

    DriverDataSnapshot getData(const DriverNames name) override;

protected:

    virtual void defineDrivers() = 0;

    PhidgetVelocityCommandDataTransformer & getCmdVelConverter();

    // Drive train system settings
    DrivetrainSettings drivetrain_settings_;

    std::unordered_map<DriverNames, std::shared_ptr<DriverInterface>> drivers_;
    
private:

    void initDrivers();

    void setMotorsStates(
        DriverDataSnapshot & data,
        const MotorDriverState & state,
        const bool data_timed_out);

    void setDriverState(
        DriverDataSnapshot & data,
        const DriverState & state,
        const bool data_timed_out);

    bool initialized_ = false;

    // Guards `data_`'s values (not its structure - entries are only ever inserted once,
    // single-threaded, in initialize()). Written from the RT thread (updateMotorsState()/
    // updateDriversState()); read from getData(), which may be called from a different thread
    // (e.g. RoverA1System::diagnoseErrors()/diagnoseStatus(), which run on
    // SystemROSInterface's own executor thread, not the RT thread).
    mutable std::mutex data_mtx_;
    std::unordered_map<DriverNames, DriverDataSnapshot> data_;

    // getData() is itself on the RT path (called every read() cycle via
    // RoverA1System::updateHwStates()), so unlike updateMotorsState()/updateDriversState() it must
    // never block waiting on the diagnostics thread's own getData() call. Mirrors `data_`'s keys
    // 1:1 (seeded alongside it in initialize(), never structurally modified afterwards) and is
    // refreshed opportunistically in getData() on uncontended lock acquisition; on contention the
    // last successfully-read snapshot is returned instead of blocking.
    std::unordered_map<DriverNames, DriverDataSnapshot> last_known_data_;

    PhidgetVelocityCommandDataTransformer phidget_vel_cmd_converter_;

    const std::chrono::milliseconds activate_wait_time_;

    // Written by updateCommunicationStatus() (RT thread), read by isCommunicationError() (may be
    // called from a different thread - see data_mtx_ above for the same cross-thread rationale).
    std::atomic_bool has_communication_error_{false};
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_ROVER_DRIVER_PHIDGET_ROVER_DRIVER_HPP_