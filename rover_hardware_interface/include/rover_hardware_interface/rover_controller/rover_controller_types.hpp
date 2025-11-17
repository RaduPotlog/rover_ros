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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_TYPES_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_TYPES_HPP_

#include <map>
#include <string>

#include "rover_hardware_interface/rover_modbus/modbus_types.hpp"

namespace rover_hardware_interface
{

enum class RoverControllerGpio
{
    GPIO_HW_E_STOP_USER_BTN             = 0,
    GPIO_1                              = 1,
    GPIO_2                              = 2,
    GPIO_3                              = 3,
    GPIO_4                              = 4,
    GPIO_5                              = 5,
    GPIO_6                              = 6,
    GPIO_7                              = 7,
    
    GPIO_MOTOR_CONTACTOR_ENGAGED        = 8,
    GPIO_SW_E_STOP_CPU_WDG_TRIGGER      = 9,  // sw_e_stop_cpu_wdg_trigger
    GPIO_SW_E_STOP_USER_BUTTON          = 10, // sw_e_stop_user_button
    GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT   = 11, // sw_e_stop_motor_driver_faults
    GPIO_SW_E_STOP_LATCH_RESET          = 12, // sw_e_stop_latch_reset
    GPIO_SW_E_STOP_LATCH_STATUS         = 13, // sw_e_stop_latch_status
    GPIO_14                             = 14,
    GPIO_15                             = 15,
};

const std::map<RoverControllerGpio, std::string> gpio_names_
{
    { RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN,             "GPIO_HW_E_STOP_USER_BTN"               },
    { RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED,        "GPIO_MOTOR_CONTACTOR_ENGAGED"          },
    { RoverControllerGpio::GPIO_SW_E_STOP_CPU_WDG_TRIGGER,      "GPIO_SW_E_STOP_CPU_WDG_TRIGGER"        },
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

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_TYPES_HPP_
