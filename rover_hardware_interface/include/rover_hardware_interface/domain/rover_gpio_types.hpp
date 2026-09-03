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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_GPIO_TYPES_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_GPIO_TYPES_HPP_

namespace rover_hardware_interface
{

// Identifies a GPIO/relay line on the safety controller. Kept in domain/ (rather than
// rover_controller/rover_controller_types.hpp, where it used to live) because RoverGpioPort
// needs it in its signature, and domain code may not include infrastructure headers - see
// scripts/check_domain_purity.sh. rover_controller_types.hpp re-includes this header so existing
// infrastructure call sites are unaffected.
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

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_GPIO_TYPES_HPP_
