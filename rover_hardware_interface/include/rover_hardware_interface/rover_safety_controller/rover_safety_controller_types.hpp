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

// Period of the CPU watchdog heartbeat toggle written to GPIO_CPU_WDG_HEARTBEAT.
//
// This is the one timing value in the system that is dimensioned against a *hardware* deadline:
// the safety relay latches the E-Stop if the heartbeat level stops changing for longer than its
// watchdog window (~1 s on the A1). It used to be a bare `sleep_for(500ms)` literal sharing a
// loop with the IO poll, which meant the real interval was "500 ms + every Modbus round-trip the
// poll made" - one response timeout was enough to overshoot the window and latch a nuisance stop
// that then needed a manual reset to clear.
//
// 200 ms gives ~5x margin on a 1 s window, so the heartbeat can miss several consecutive ticks
// to IO contention and still keep the relay fed.
constexpr unsigned kDefaultWdgKickPeriodMs = 200;

// Period of the discrete-IO poll that refreshes the cache read() serves from. Telemetry only:
// it feeds GpioState and the E-Stop mirror, and it must never be allowed to delay the heartbeat
// above. 100 ms (10 Hz) keeps the link duty cycle low while staying well inside the 1.0 s
// staleness timeout rover_twist_mux applies to gpio_state.
constexpr unsigned kDefaultIoPollPeriodMs = 100;

// How long the latch-reset coil is held true before being driven back to false.
//
// The reset is a pulse, not a level. It used to be written true then false back-to-back with no
// dwell at all, which made the pulse exactly as wide as one Modbus round-trip - a few
// milliseconds, and not a width anyone had checked against the relay's input filter. A pulse the
// relay cannot see fails in the most confusing way available: the service returns success and
// the latch stays held.
//
// 100 ms is comfortably wider than any ordinary discrete-input filter. It costs nothing: the
// reset runs on a non-RT service-callback thread, and an operator clearing a latched E-Stop is
// not counting milliseconds. Narrow it only against a measurement.
constexpr unsigned kDefaultLatchResetPulseMs = 100;

// Timing for the safety controller's two background threads. Parsed out of the URDF
// <ros2_control> hardware parameters by RoverA1System, like ModbusSettings.
struct SafetyControllerSettings
{
    unsigned wdg_kick_period_ms = kDefaultWdgKickPeriodMs;
    unsigned io_poll_period_ms = kDefaultIoPollPeriodMs;
    unsigned latch_reset_pulse_ms = kDefaultLatchResetPulseMs;
};

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
