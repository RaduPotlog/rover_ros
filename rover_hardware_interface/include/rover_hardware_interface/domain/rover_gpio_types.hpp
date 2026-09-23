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
// rover_safety_controller/rover_safety_controller_types.hpp, where it used to live) because
// RoverGpioPort needs it in its signature, and domain code may not include infrastructure
// headers - see scripts/check_domain_purity.sh. rover_safety_controller_types.hpp re-includes
// this header so existing infrastructure call sites are unaffected.
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
    GPIO_CPU_WDG_HEARTBEAT              = 9,  // cpu_wdg_heartbeat (periodic toggle, not a fault)
    GPIO_SW_E_STOP_USER_BUTTON          = 10,  // sw_e_stop_user_button
    GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT   = 11,  // sw_e_stop_motor_driver_faults
    GPIO_SW_E_STOP_LATCH_RESET          = 12,  // sw_e_stop_latch_reset
    GPIO_SW_E_STOP_LATCH_STATUS         = 13,  // sw_e_stop_latch_status
    GPIO_14                             = 14,
    GPIO_15                             = 15,

    // General-purpose aux IO on the PLC's programmable digital I/O. Not part of the safety chain:
    // nothing here may feed E-Stop or motion-inhibit logic.
    GPIO_AUX_OUT_0                      = 16,  // DIO00
    GPIO_AUX_OUT_1                      = 17,  // DIO01
    GPIO_AUX_OUT_2                      = 18,  // DIO02
    GPIO_AUX_OUT_3                      = 19,  // DIO03
    GPIO_AUX_OUT_4                      = 20,  // DIO04
    GPIO_AUX_OUT_5                      = 21,  // DIO05
    GPIO_AUX_IN_0                       = 22,  // DIO06
    GPIO_AUX_IN_1                       = 23,  // DIO07
    GPIO_AUX_IN_2                       = 24,  // DIO08
    GPIO_AUX_IN_3                       = 25,  // DIO09
    GPIO_AUX_IN_4                       = 26,  // DIO10
    GPIO_AUX_IN_5                       = 27,  // DIO11
};

constexpr unsigned kAuxOutputCount = 6;
constexpr unsigned kAuxInputCount = 6;

// GPIO_AUX_OUT_<index> / GPIO_AUX_IN_<index>; index must be below the matching count.
constexpr RoverControllerGpio auxOutputPin(const unsigned index)
{
    return static_cast<RoverControllerGpio>(
        static_cast<unsigned>(RoverControllerGpio::GPIO_AUX_OUT_0) + index);
}

constexpr RoverControllerGpio auxInputPin(const unsigned index)
{
    return static_cast<RoverControllerGpio>(
        static_cast<unsigned>(RoverControllerGpio::GPIO_AUX_IN_0) + index);
}

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_GPIO_TYPES_HPP_
