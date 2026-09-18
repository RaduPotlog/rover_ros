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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_SAFETY_CONTROLLER_ROVER_SAFETY_CONTROLLER_TYPES_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_SAFETY_CONTROLLER_ROVER_SAFETY_CONTROLLER_TYPES_HPP_

#include <map>
#include <string>

#include "rover_hardware_interface/domain/rover_gpio_types.hpp"
#include "rover_modbus_driver/domain/client_settings.hpp"
#include "rover_modbus_driver/domain/contact_coil_types.hpp"
#include "rover_modbus_driver/domain/discrete_io_port.hpp"

namespace rover_hardware_interface
{

// The Modbus client moved out to rover_transport/rover_modbus_driver, where it can be reused
// and tested on its own. These aliases keep the ~40 unqualified call sites in
// rover_safety_controller.cpp's coil table compiling unchanged, in the same spirit as the
// RoverControllerGpio re-include noted below. Prefer the qualified
// rover::transport::modbus:: names in new code.
using rover::transport::modbus::Coil;
using rover::transport::modbus::CoilInfo;
using rover::transport::modbus::Contact;
using rover::transport::modbus::ContactInfo;

using DiscreteIoPort = rover::transport::modbus::DiscreteIoPort;

// Was RoverModbusInterface before the extraction.
using RoverModbusInterface = rover::transport::modbus::DiscreteIoPort;

// Was ModbusSettings before the extraction. Still parsed out of the URDF <ros2_control>
// hardware parameters by RoverA1System - that stays a ros2_control plugin concern.
using ModbusSettings = rover::transport::modbus::ClientSettings;

// RoverControllerGpio itself now lives in domain/rover_gpio_types.hpp (RoverGpioPort needs it and
// domain code may not include infrastructure headers); re-included here so existing call sites
// that reach it via this header keep compiling unchanged.

const std::map<RoverControllerGpio, std::string> gpio_names_
{
    { RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN,             "GPIO_HW_E_STOP_USER_BTN"               },
    { RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED,        "GPIO_MOTOR_CONTACTOR_ENGAGED"          },
    { RoverControllerGpio::GPIO_CPU_WDG_HEARTBEAT,              "GPIO_CPU_WDG_HEARTBEAT"                },
    { RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON,          "GPIO_SW_E_STOP_USER_BUTTON"            },
    { RoverControllerGpio::GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT,   "GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT"     },
    { RoverControllerGpio::GPIO_SW_E_STOP_LATCH_RESET,          "GPIO_SW_E_STOP_LATCH_RESET"            },
    { RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS,         "GPIO_SW_E_STOP_LATCH_STATUS"           }
};

struct RoverControllerGpioInfo
{
    const RoverControllerGpio pin;
    bool value;
};

struct RoverControllerContactInfo
{
    const RoverControllerGpio pin;
    const ContactInfo contact_info;
};

struct RoverControllerCoilInfo
{
    const RoverControllerGpio pin;
    const CoilInfo coil_info;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_SAFETY_CONTROLLER_ROVER_SAFETY_CONTROLLER_TYPES_HPP_
