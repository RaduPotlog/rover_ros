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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_MODBUS_TYPES_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_MODBUS_TYPES_HPP_

#include <string>

namespace rover_hardware_interface
{

enum class Contact
{
    CONTACT_0 = 0,
    CONTACT_1,
    CONTACT_2,
    CONTACT_3,
    CONTACT_4,
    CONTACT_5,
    CONTACT_6,
    CONTACT_7,
};

enum class Coil
{
    COIL_0 = 0,
    COIL_1,
    COIL_2,
    COIL_3,
    COIL_4,
    COIL_5,
    COIL_6,
    COIL_7,
};

struct ContactInfo
{
    Contact contact;
};

struct CoilInfo
{
    const Coil coil;
    const bool default_coil_state;
    const bool is_coil_engage_allowed;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_MODBUS_TYPES_HPP_
