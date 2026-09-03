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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_DRIVER_HPP_

#include <string>
#include <vector>

#include "rover_hardware_interface/domain/driver.hpp"

#include "rover_hardware_interface/domain/driver_data_snapshot.hpp"

namespace rover_hardware_interface
{

class RoverDriverInterface
{

public:

    virtual void initialize() = 0;

    virtual void deinitialize() = 0;

    virtual void activate() = 0;

    virtual void updateCommunicationStatus() = 0;

    virtual void updateMotorsState() = 0;

    virtual void updateDriversState() = 0;

    // Returned by value (a copy taken under lock by the implementation): getData() may be called
    // from a thread other than the one that mutates the driver's state (e.g. diagnostics), so a
    // reference into internal state would not be safe to hand out.
    virtual DriverDataSnapshot getData(const DriverNames name) = 0;

    virtual void sendSpeedCmd(const std::vector<float> & speeds) = 0;

    virtual void attemptErrorFlagReset() = 0;

    virtual bool isCommunicationError() = 0;

    virtual bool isMotorStatesDataTimedOut() = 0;

    virtual bool isDriverStateDataTimedOut() = 0;

    virtual bool isFlagError() = 0;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_DRIVER_HPP_
