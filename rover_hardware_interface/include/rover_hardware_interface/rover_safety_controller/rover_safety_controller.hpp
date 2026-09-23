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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_SAFETY_CONTROLLER_ROVER_SAFETY_CONTROLLER_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_SAFETY_CONTROLLER_ROVER_SAFETY_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rover_hardware_interface/domain/safety_link_health.hpp"
#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_types.hpp"
#include "rover_modbus_driver/domain/client_settings.hpp"
#include "rover_modbus_driver/domain/discrete_io_port.hpp"
#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

// Serializes access to the single Modbus link, with priority for the CPU watchdog heartbeat and
// the E-Stop commands over the periodic IO poll.
//
// A plain mutex - even a timed one - is not sufficient here. Under sustained slow IO the poll
// thread releases and immediately re-acquires between transactions, and neither std::mutex nor
// std::timed_mutex makes any fairness guarantee, so a heartbeat tick can lose the race
// arbitrarily many times in a row. That is precisely the failure this whole rework exists to
// prevent. With this lock a waiting priority holder blocks *new* poll acquisitions, so the
// heartbeat waits at most one already-in-flight transaction.
class ModbusLink
{

public:

    // Blocks until the link is free. Priority acquirers also overtake any queued poll acquirers.
    void acquire(const bool priority);

    void release();

private:

    std::mutex mtx_;
    std::condition_variable cv_;
    bool busy_ = false;
    unsigned priority_waiters_ = 0;
};

// RAII holder for ModbusLink.
class ModbusLinkGuard
{

public:

    ModbusLinkGuard(ModbusLink & link, const bool priority)
    : link_(link), held_(true)
    {
        link_.acquire(priority);
    }

    ~ModbusLinkGuard()
    {
        if (held_) {
            link_.release();
        }
    }

    ModbusLinkGuard(const ModbusLinkGuard &) = delete;
    ModbusLinkGuard & operator=(const ModbusLinkGuard &) = delete;

private:

    ModbusLink & link_;
    bool held_;
};

// Depends on DiscreteIoPort (not the concrete ModbusDiscreteIoClient) so it can be unit-tested
// with a fake - see rover_modbus_driver/domain/discrete_io_port.hpp and
// test/rover_safety_controller/.
//
// Threading: the heartbeat and the IO poll run on *separate* threads with separate periods.
// They used to share one loop, which made the heartbeat interval "poll period + every Modbus
// round-trip the poll performed" - a single response timeout pushed the toggle past the relay's
// ~1 s watchdog window and latched the E-Stop. The heartbeat is safety-critical timing; the poll
// is telemetry. They must not share a period, a deadline, or a lock hold.
class ContactCoilHandler
{

public:

    ContactCoilHandler(
        std::shared_ptr<DiscreteIoPort> rover_modbus,
        const SafetyControllerSettings & settings = SafetyControllerSettings {});

    ~ContactCoilHandler();

    bool start();

    bool isContactCoilHandlerEnabled() const;

    // SW E-STOP USER BTN - sw_e_stop_user_button
    void eStopUserBtnTrigger(const bool state);

    // SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
    void eStopMotorDriverFaultTrigger(const bool state);

    // SW E-STOP LATCH RESET - sw_e_stop_latch_reset
    void eStopLatchReset();

    // Drives GPIO_AUX_OUT_<index>. Blocks for one Modbus write, taking the link without priority.
    // Throws std::out_of_range for index >= kAuxOutputCount, and whatever the write throws.
    void setAuxOutput(const unsigned index, const bool state);

    // Fills `io_state` from the last-polled IO state. Non-blocking: on lock contention with the
    // poll thread, `io_state` is left unchanged (i.e. the caller's own last-known-good values),
    // so this is safe to call from the RT thread. Contends only with a map copy (io_state_mtx_),
    // never with a Modbus transaction.
    void getIoState(std::unordered_map<RoverControllerGpio, bool> & io_state);

    SafetyLinkHealth getHealth() const;

private:

    void initCoils();

    // Performs the Modbus reads - one batched transaction for the contacts, one for the coils -
    // taking modbus_link_ once per transaction rather than once for the whole sweep, so a
    // heartbeat tick waits at most one round-trip to reach the link.
    std::unordered_map<RoverControllerGpio, bool> queryControlInterfaceIOStates();

    void contactCoilHandlerWatchdogThread();

    void contactCoilHandlerPollThread();

    static uint64_t steadyNowMs();

    std::thread contact_coil_handler_watchdog_thread_;
    std::thread contact_coil_handler_poll_thread_;
    std::atomic_bool contact_coil_handler_enabled_ = false;

    std::shared_ptr<DiscreteIoPort> rover_modbus_;

    const SafetyControllerSettings settings_;

    static const std::vector<RoverControllerContactInfo> contacts_config_info_storage_;
    static const std::vector<RoverControllerCoilInfo> coils_config_info_storage_;

    // Serializes access to the Modbus link, with heartbeat/E-Stop priority over the IO poll.
    ModbusLink modbus_link_;

    // Guards io_state_ only. Held for a map copy, never across a Modbus transaction, which is
    // what keeps getIoState()'s try_lock succeeding on the RT thread.
    mutable std::mutex io_state_mtx_;

    std::unordered_map<RoverControllerGpio, bool> io_state_;

    // Heartbeat level for GPIO_CPU_WDG_HEARTBEAT, flipped once per heartbeat tick. Touched only
    // by contactCoilHandlerWatchdogThread(), so it needs no lock of its own.
    bool wdg_state_ = false;

    std::atomic_uint64_t last_kick_ms_ {0};
    std::atomic_uint64_t last_poll_ms_ {0};
    std::atomic_uint64_t wdg_miss_count_ {0};
    std::atomic_uint64_t wdg_error_count_ {0};
    std::atomic_uint64_t poll_error_count_ {0};
    std::atomic_bool wdg_thread_running_ {false};
    std::atomic_bool poll_thread_running_ {false};
};

class RoverSafetyController
{

public:

    explicit RoverSafetyController(
        const ModbusSettings & modbus_settings,
        const SafetyControllerSettings & settings = SafetyControllerSettings {});

    // Test-only constructor: injects rover_modbus directly instead of dialing a real Modbus TCP
    // endpoint, so RoverSafetyController's/ContactCoilHandler's coil-mapping and enabled-guard
    // logic can be unit-tested against a fake DiscreteIoPort (see
    // test/rover_safety_controller/). Production code always uses the constructor above.
    explicit RoverSafetyController(
        std::shared_ptr<DiscreteIoPort> rover_modbus,
        const SafetyControllerSettings & settings = SafetyControllerSettings {});

    // Start resources and ContactCoilHandler threads
    void start();

    // SW E-STOP USER BTN - sw_e_stop_user_button
    void eStopUserBtnTrigger(const bool state);

    // SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
    void eStopMotorDriverFaultTrigger(const bool state);

    // SW E-STOP LATCH RESET - sw_e_stop_latch_reset
    void eStopLatchReset();

    // Drives GPIO_AUX_OUT_<index>. Not RT-safe (one blocking Modbus write). Throws if called
    // before start(), for an out-of-range index, or when the write fails.
    void setAuxOutput(const unsigned index, const bool state);

    // Non-blocking; returns a reference to a cache owned by this RoverSafetyController, refreshed
    // in place on each call (best-effort - see ContactCoilHandler::getIoState()). Safe to call
    // from the RT thread; avoids allocating a new map on every call.
    const std::unordered_map<RoverControllerGpio, bool> & queryControlInterfaceIOStates();

    bool isPinActive(const RoverControllerGpio pin);

    // Background-thread health for the diagnostics task. Returns a default-constructed (all
    // stopped) snapshot before start().
    SafetyLinkHealth getHealth() const;

private:

    std::unique_ptr<ContactCoilHandler> contactCoilHandler_;

    std::shared_ptr<DiscreteIoPort> rover_modbus_;

    const SafetyControllerSettings settings_;

    // Reused across calls by queryControlInterfaceIOStates() to avoid allocating on the RT thread.
    std::unordered_map<RoverControllerGpio, bool> io_state_cache_;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_SAFETY_CONTROLLER_ROVER_SAFETY_CONTROLLER_HPP_
