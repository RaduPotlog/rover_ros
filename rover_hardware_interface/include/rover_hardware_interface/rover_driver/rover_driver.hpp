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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_ROVER_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_ROVER_DRIVER_HPP_

#include <string>
#include <vector>

#include "rover_hardware_interface/rover_driver/driver.hpp"

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_data_transformer.hpp"

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

    virtual const PhidgetDriverDataTransformer & getData(const DriverNames name) = 0;

    virtual void sendSpeedCmd(const std::vector<float> & speeds) = 0;

    virtual void attemptErrorFlagReset() = 0;

    virtual bool isCommunicationError() = 0;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_ROVER_DRIVER_HPP_
