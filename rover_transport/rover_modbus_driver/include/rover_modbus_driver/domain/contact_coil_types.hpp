// Copyright 2026 Mechatronics Academy
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

#ifndef ROVER_MODBUS_DRIVER_DOMAIN_CONTACT_COIL_TYPES_HPP_
#define ROVER_MODBUS_DRIVER_DOMAIN_CONTACT_COIL_TYPES_HPP_

namespace rover::transport::modbus
{

// The enum value IS the Modbus address - see ModbusDiscreteIoClient, which casts it
// straight into the request. Moved here from rover_hardware_interface's modbus_types.hpp.

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
    // Modified 2026 by Mechatronics Academy: dropped the const qualifiers these three
    // members carried. They made CoilInfo non-assignable, which is why the safety
    // controller's coil table could only ever be brace-initialised. The struct is passed
    // by const reference everywhere, so the constness bought nothing.
    Coil coil;
    bool default_coil_state;
    bool is_coil_engage_allowed;
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_DOMAIN_CONTACT_COIL_TYPES_HPP_
