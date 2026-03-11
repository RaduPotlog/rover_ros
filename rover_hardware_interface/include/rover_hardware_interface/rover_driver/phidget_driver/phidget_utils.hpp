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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_DRIVER_PHIDGET_UTILS_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_DRIVER_PHIDGET_UTILS_HPP_

#include <stdexcept>
#include <string>

#include <libphidget22/phidget22.h>

namespace rover_hardware_interface
{

#define PHIDGET_UTILS_NO_COPY_NO_MOVE_NO_ASSIGN(Classname)  \
    Classname(const Classname &) = delete;                  \
    void operator=(const Classname &) = delete;             \
    Classname(Classname &&) = delete;                       \
    void operator=(Classname &&) = delete;

void openWaitForAttachment(
    PhidgetHandle handle, 
    int32_t serial_number,
    int hub_port, 
    bool is_hub_port_device, 
    int channel);

void closeAndDelete(PhidgetHandle * handle) noexcept;

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_DRIVER_PHIDGET_UTILS_HPP_