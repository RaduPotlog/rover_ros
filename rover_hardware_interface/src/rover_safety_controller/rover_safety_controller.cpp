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

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_types.hpp"

#include "rover_modbus_driver/infrastructure/modbus_tcp_client_factory.hpp"

namespace rover_hardware_interface
{

const std::vector<RoverControllerContactInfo> ContactCoilHandler::contacts_config_info_storage_ = {

    RoverControllerContactInfo {
        RoverControllerGpio {RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN},
        ContactInfo { Contact::CONTACT_0 },
    }
};

const std::vector<RoverControllerCoilInfo> ContactCoilHandler::coils_config_info_storage_ = {

    RoverControllerCoilInfo {
        RoverControllerGpio {RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED},
        CoilInfo { Coil::COIL_0, false, false},
    },

    RoverControllerCoilInfo {
        RoverControllerGpio {RoverControllerGpio::GPIO_CPU_WDG_HEARTBEAT},
        CoilInfo { Coil::COIL_1, true, true},
    },

    RoverControllerCoilInfo {
        RoverControllerGpio {RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON},
        CoilInfo { Coil::COIL_2, true, true},
    },

    RoverControllerCoilInfo {
        RoverControllerGpio {RoverControllerGpio::GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT},
        CoilInfo { Coil::COIL_3, true, true},
    },

    RoverControllerCoilInfo {
        RoverControllerGpio {RoverControllerGpio::GPIO_SW_E_STOP_LATCH_RESET},
        CoilInfo { Coil::COIL_4, false, true},
    },

    RoverControllerCoilInfo {
        RoverControllerGpio {RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS},
        CoilInfo { Coil::COIL_5, false, false},
    },

    // General-purpose aux IO on the PLC's programmable digital I/O (DIO00..DIO11). Outputs are
    // driven OFF by initCoils() on every start; inputs are PLC-owned and marked non-engageable so
    // the driver refuses any write to them.
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_OUT_0, CoilInfo { Coil::COIL_8,  false, true  } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_OUT_1, CoilInfo { Coil::COIL_9,  false, true  } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_OUT_2, CoilInfo { Coil::COIL_10, false, true  } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_OUT_3, CoilInfo { Coil::COIL_11, false, true  } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_OUT_4, CoilInfo { Coil::COIL_12, false, true  } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_OUT_5, CoilInfo { Coil::COIL_13, false, true  } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_IN_0,  CoilInfo { Coil::COIL_14, false, false } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_IN_1,  CoilInfo { Coil::COIL_15, false, false } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_IN_2,  CoilInfo { Coil::COIL_16, false, false } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_IN_3,  CoilInfo { Coil::COIL_17, false, false } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_IN_4,  CoilInfo { Coil::COIL_18, false, false } },
    RoverControllerCoilInfo { RoverControllerGpio::GPIO_AUX_IN_5,  CoilInfo { Coil::COIL_19, false, false } },
};

// Indices into coils_config_info_storage_ for the coils addressed by name below. Spelled out
// rather than used as bare literals: the watchdog coil in particular is a heartbeat output, not a
// stop condition, and an unlabelled `[1]` hides that.
constexpr std::size_t kCpuWdgHeartbeatCoilIdx = 1;
constexpr std::size_t kEStopUserBtnCoilIdx = 2;
constexpr std::size_t kEStopMotorDriverFaultCoilIdx = 3;
constexpr std::size_t kEStopLatchResetCoilIdx = 4;
constexpr std::size_t kFirstAuxOutputCoilIdx = 6;

// Number of consecutive objects, starting at address 0, that one batched read must span to cover
// every entry of a table.
template <typename InfoT, typename AddressOf>
uint16_t spanFromZero(const std::vector<InfoT> & table, AddressOf address_of)
{
    uint16_t span = 0;

    for (const auto & entry : table) {
        span = std::max<uint16_t>(span, static_cast<uint16_t>(address_of(entry) + 1));
    }

    return span;
}

void ModbusLink::acquire(const bool priority)
{
    std::unique_lock<std::mutex> lck(mtx_);

    if (priority) {
        priority_waiters_++;
    }

    // A poll acquirer additionally waits for priority_waiters_ to drain, which is what stops it
    // from repeatedly winning the race against a heartbeat tick.
    cv_.wait(lck, [this, priority] {
        return !busy_ && (priority || priority_waiters_ == 0);
    });

    if (priority) {
        priority_waiters_--;
    }

    busy_ = true;
}

void ModbusLink::release()
{
    {
        std::lock_guard<std::mutex> lck(mtx_);
        busy_ = false;
    }

    cv_.notify_all();
}

ContactCoilHandler::ContactCoilHandler(
    std::shared_ptr<DiscreteIoPort> rover_modbus, const SafetyControllerSettings & settings)
: rover_modbus_(std::move(rover_modbus)), settings_(settings)
{

}

ContactCoilHandler::~ContactCoilHandler()
{
    contact_coil_handler_enabled_ = false;

    if (contact_coil_handler_watchdog_thread_.joinable()) {
        contact_coil_handler_watchdog_thread_.join();
    }

    if (contact_coil_handler_poll_thread_.joinable()) {
        contact_coil_handler_poll_thread_.join();
    }
}

uint64_t ContactCoilHandler::steadyNowMs()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool ContactCoilHandler::start()
{
    if (isContactCoilHandlerEnabled()) {
        return true;
    }

    initCoils();

    contact_coil_handler_enabled_ = true;

    // The heartbeat goes up first and independently: if the poll thread cannot be started, or
    // later dies, the relay must still be fed rather than latch a stop the operator has to clear
    // by hand.
    contact_coil_handler_watchdog_thread_ =
        std::thread(&ContactCoilHandler::contactCoilHandlerWatchdogThread, this);
    contact_coil_handler_poll_thread_ =
        std::thread(&ContactCoilHandler::contactCoilHandlerPollThread, this);

    return isContactCoilHandlerEnabled();
}

bool ContactCoilHandler::isContactCoilHandlerEnabled() const
{
    return contact_coil_handler_watchdog_thread_.joinable() &&
           contact_coil_handler_poll_thread_.joinable();
}

// SW E-STOP USER BTN - sw_e_stop_user_button
void ContactCoilHandler::eStopUserBtnTrigger(const bool state)
{
    // Priority: an E-Stop command must not queue behind a sweep of slow IO reads.
    ModbusLinkGuard lck(modbus_link_, true);
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[kEStopUserBtnCoilIdx].coil_info, state);
}

// SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
void ContactCoilHandler::eStopMotorDriverFaultTrigger(const bool state)
{
    ModbusLinkGuard lck(modbus_link_, true);
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[kEStopMotorDriverFaultCoilIdx].coil_info, state);
}

// SW E-STOP LATCH RESET - sw_e_stop_latch_reset
//
// A pulse, held for settings_.latch_reset_pulse_ms. The two writes used to be back-to-back, which
// made the pulse one Modbus round-trip wide; see kDefaultLatchResetPulseMs for why that is not a
// width to leave unchecked. Sleeping here is safe - this runs on a non-RT service-callback thread
// (MutuallyExclusive), never on the read()/write() path.
//
// The link is held for the whole dwell on purpose: releasing it mid-pulse would let the IO poll
// read the coil back and report a reset that is still in progress. The heartbeat has priority and
// will be a dwell late at worst, which is far inside the relay's window.
void ContactCoilHandler::eStopLatchReset()
{
    ModbusLinkGuard lck(modbus_link_, true);

    const auto & coil = coils_config_info_storage_[kEStopLatchResetCoilIdx].coil_info;

    rover_modbus_->writeDiscreteCoil(coil, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(settings_.latch_reset_pulse_ms));
    rover_modbus_->writeDiscreteCoil(coil, false);
}

// General-purpose output, not a safety command: non-priority on the link, so it queues behind the
// heartbeat and any E-Stop write exactly like an IO poll transaction does.
void ContactCoilHandler::setAuxOutput(const unsigned index, const bool state)
{
    if (index >= kAuxOutputCount) {
        throw std::out_of_range(
            "Aux output index " + std::to_string(index) + " out of range (0.." +
            std::to_string(kAuxOutputCount - 1) + ").");
    }

    ModbusLinkGuard lck(modbus_link_, false);
    rover_modbus_->writeDiscreteCoil(
        coils_config_info_storage_[kFirstAuxOutputCoilIdx + index].coil_info, state);
}

void ContactCoilHandler::getIoState(std::unordered_map<RoverControllerGpio, bool> & io_state)
{
    if (io_state_mtx_.try_lock()) {
        std::lock_guard<std::mutex> io_state_lck(io_state_mtx_, std::adopt_lock);
        // Update values in place instead of `io_state = io_state_` (a copy-assignment):
        // std::unordered_map stores each element as a separately heap-allocated node, and a
        // copy-assignment is not guaranteed allocation-free even at equal size. Once `io_state`
        // already holds every key (true from the second call onward, since the key set is fixed),
        // operator[] here only updates existing nodes and never allocates, keeping this call
        // RT-safe on the read() path.
        for (const auto & [pin, value] : io_state_) {
            io_state[pin] = value;
        }
    }
    // else: leave `io_state` as-is (the caller's last-known-good values) rather than clobber it
    // with an empty map on contention. io_state_mtx_ is only ever held for this copy, never
    // across a Modbus transaction, so contention here is brief by construction.
}

SafetyLinkHealth ContactCoilHandler::getHealth() const
{
    SafetyLinkHealth health;

    health.watchdog_running = wdg_thread_running_.load();
    health.poll_running = poll_thread_running_.load();
    health.watchdog_miss_count = wdg_miss_count_.load();
    health.watchdog_error_count = wdg_error_count_.load();
    health.poll_error_count = poll_error_count_.load();

    const uint64_t now_ms = steadyNowMs();
    const uint64_t last_kick = last_kick_ms_.load();
    const uint64_t last_poll = last_poll_ms_.load();

    health.last_kick_age_ms =
        (last_kick == 0) ? SafetyLinkHealth::kUnknownAgeMs : (now_ms - last_kick);
    health.last_poll_age_ms =
        (last_poll == 0) ? SafetyLinkHealth::kUnknownAgeMs : (now_ms - last_poll);

    return health;
}

// Drives every *writable* coil to its documented default. COIL_0 (motor contactor) and COIL_5
// (latch status) are outputs of the relay that we only ever read: their is_coil_engage_allowed is
// false, ModbusDiscreteIoClient refuses the write and logs it at error level, so looping over
// them unconditionally used to emit two spurious "Coil engage is not allowed" errors on every
// successful start. Skipping them here keeps a clean start genuinely clean, which matters because
// those logs are the first thing anyone looks at when the safety link misbehaves.
void ContactCoilHandler::initCoils()
{
    for (size_t i = 0; i < coils_config_info_storage_.size(); i++) {
        const auto & coil_info = coils_config_info_storage_[i].coil_info;

        if (!coil_info.is_coil_engage_allowed) {
            continue;
        }

        rover_modbus_->writeDiscreteCoil(coil_info, coil_info.default_coil_state);
    }
}

// Two transactions per sweep regardless of how many points are mapped: one FC2 read across the
// contacts, one FC1 read across the coils, each spanning address 0 up to the highest address in
// its table. With single-bit reads the sweep cost one round-trip per point (7, then 19 with the
// aux IO), which on a slow link could age the poll past kSafetyLinkStalePollAgeMs.
//
// A failed read throws out of the sweep; the poll thread counts it and io_state_ keeps its
// previous values.
std::unordered_map<RoverControllerGpio, bool> ContactCoilHandler::queryControlInterfaceIOStates()
{
    static const uint16_t contact_span = spanFromZero(
        contacts_config_info_storage_,
        [](const RoverControllerContactInfo & c) { return static_cast<uint16_t>(c.contact_info.contact); });
    static const uint16_t coil_span = spanFromZero(
        coils_config_info_storage_,
        [](const RoverControllerCoilInfo & c) { return static_cast<uint16_t>(c.coil_info.coil); });

    std::vector<bool> contact_bits;
    std::vector<bool> coil_bits;

    {
        // One lock hold per transaction, not one for the whole sweep: a heartbeat tick waiting
        // on the link then waits at most a single round-trip. Non-priority, so a waiting
        // heartbeat overtakes the next read.
        ModbusLinkGuard lck(modbus_link_, false);
        contact_bits = rover_modbus_->readDiscreteContacts(Contact::CONTACT_0, contact_span);
    }

    {
        ModbusLinkGuard lck(modbus_link_, false);
        coil_bits = rover_modbus_->readDiscreteCoils(Coil::COIL_0, coil_span);
    }

    std::unordered_map<RoverControllerGpio, bool> io_state;

    for (const auto & contact : contacts_config_info_storage_) {
        io_state.emplace(contact.pin, contact_bits.at(static_cast<size_t>(contact.contact_info.contact)));
    }

    for (const auto & coil : coils_config_info_storage_) {
        io_state.emplace(coil.pin, coil_bits.at(static_cast<size_t>(coil.coil_info.coil)));
    }

    return io_state;
}

// Kick the safety relay's CPU watchdog. The relay watches for a *changing* level, so this coil is
// driven as a square wave: it is a liveness heartbeat, never a fault flag. Anything downstream
// that reads it back (see GpioState.gpio_pin_cpu_wdg_heartbeat) must not treat either level as a
// stop condition - a stalled heartbeat is caught by the relay itself, which latches the e-stop
// (sw_e_stop_latch_status).
//
// The loop runs on absolute steady_clock deadlines, not sleep_for after the work, so the toggle
// interval is settings_.wdg_kick_period_ms regardless of how long a Modbus write takes. A tick
// that cannot take the link before its own deadline is counted as a miss and retried on the next
// deadline rather than blocking: falling behind by one tick is recoverable, blocking past the
// relay's window is not.
void ContactCoilHandler::contactCoilHandlerWatchdogThread()
{
    const auto period = std::chrono::milliseconds(settings_.wdg_kick_period_ms);
    auto next_deadline = std::chrono::steady_clock::now();

    wdg_thread_running_ = true;

    while (contact_coil_handler_enabled_) {
        next_deadline += period;

        try {
            // Unbounded, deliberately. An earlier version gave up if it could not take the link
            // by its own next deadline, which made things worse rather than better: when one
            // transaction is slower than one kick period the tick would abandon the wait moments
            // before winning, sleep a period, and race again - the heartbeat interval stretched
            // to several periods under exactly the slow-IO conditions it has to survive.
            //
            // Because this acquirer has priority, the poll cannot start a further transaction
            // while we queue, so the wait is bounded by the one already in flight - i.e. by the
            // Modbus response timeout, which is configured well under the kick period. Waiting
            // for that is strictly better than retrying, since there is no kick to be had
            // without the link anyway.
            ModbusLinkGuard lck(modbus_link_, true);

            rover_modbus_->writeDiscreteCoil(
                coils_config_info_storage_[kCpuWdgHeartbeatCoilIdx].coil_info, wdg_state_);
            wdg_state_ = !wdg_state_;
            last_kick_ms_ = steadyNowMs();

            if (std::chrono::steady_clock::now() > next_deadline) {
                // Landed late. Not fatal on its own - the relay tolerates several periods of
                // slack - but a rising count is the early warning that the safety link is too
                // slow for the configured margin, so diagnostics surfaces it.
                wdg_miss_count_++;
            }
        } catch (const std::exception & e) {
            // An exception escaping a std::thread calls std::terminate() and takes the whole
            // ros2_control_node with it. Swallow, count, and keep kicking: the next tick may
            // well succeed, and if it does not the relay latches on its own - which is the
            // designed failure mode, unlike a dead process.
            wdg_error_count_++;
            std::cerr << "Safety controller watchdog kick failed: " << e.what() << std::endl;
        } catch (...) {
            wdg_error_count_++;
            std::cerr << "Safety controller watchdog kick failed: unknown exception" << std::endl;
        }

        const auto now = std::chrono::steady_clock::now();

        if (next_deadline < now) {
            // Overran. Re-base rather than burst-catching up on missed ticks - a burst of writes
            // would not help the relay, which only cares about the interval between edges. The
            // miss itself was already counted above.
            next_deadline = now + period;
        }

        std::this_thread::sleep_until(next_deadline);
    }

    wdg_thread_running_ = false;
}

// Refreshes the IO cache read() serves from. Telemetry: a missed poll degrades freshness (and is
// caught downstream by rover_twist_mux's gpio_state staleness timeout), it never endangers the
// relay's watchdog window, which is why this is a separate thread from the heartbeat above.
void ContactCoilHandler::contactCoilHandlerPollThread()
{
    const auto period = std::chrono::milliseconds(settings_.io_poll_period_ms);
    auto next_deadline = std::chrono::steady_clock::now();

    poll_thread_running_ = true;

    while (contact_coil_handler_enabled_) {
        next_deadline += period;

        try {
            // Built outside io_state_mtx_ so that lock is only ever held for the swap below,
            // never across the Modbus reads.
            auto fresh_io_state = queryControlInterfaceIOStates();

            {
                std::lock_guard<std::mutex> io_state_lck(io_state_mtx_);
                io_state_ = std::move(fresh_io_state);
            }

            last_poll_ms_ = steadyNowMs();
        } catch (const std::exception & e) {
            poll_error_count_++;
            std::cerr << "Safety controller IO poll failed: " << e.what() << std::endl;
        } catch (...) {
            poll_error_count_++;
            std::cerr << "Safety controller IO poll failed: unknown exception" << std::endl;
        }

        const auto now = std::chrono::steady_clock::now();

        if (next_deadline < now) {
            next_deadline = now + period;
        }

        std::this_thread::sleep_until(next_deadline);
    }

    poll_thread_running_ = false;
}

RoverSafetyController::RoverSafetyController(
    const ModbusSettings & modbus_settings, const SafetyControllerSettings & settings)
: rover_modbus_(rover::transport::modbus::makeModbusTcpDiscreteIoClient(modbus_settings)),
  settings_(settings)
{

}

RoverSafetyController::RoverSafetyController(
    std::shared_ptr<DiscreteIoPort> rover_modbus, const SafetyControllerSettings & settings)
: rover_modbus_(std::move(rover_modbus)), settings_(settings)
{

}

void RoverSafetyController::start()
{
    contactCoilHandler_ = std::make_unique<ContactCoilHandler>(rover_modbus_, settings_);
    contactCoilHandler_->start();
}

// SW E-STOP USER BTN - sw_e_stop_user_button
void RoverSafetyController::eStopUserBtnTrigger(const bool state)
{
    // contactCoilHandler_ is null until start() is called - guard against that first (rather
    // than dereferencing it to ask isContactCoilHandlerEnabled()) so calling this before start()
    // is a safe no-op instead of a null-pointer dereference.
    if (!contactCoilHandler_ || !contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopUserBtnTrigger(state);
}

// SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
void RoverSafetyController::eStopMotorDriverFaultTrigger(const bool state)
{
    if (!contactCoilHandler_ || !contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopMotorDriverFaultTrigger(state);
}

// SW E-STOP LATCH RESET - sw_e_stop_latch_reset
void RoverSafetyController::eStopLatchReset()
{
    if (!contactCoilHandler_ || !contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopLatchReset();
}

void RoverSafetyController::setAuxOutput(const unsigned index, const bool state)
{
    // Unlike the E-Stop triggers this must not be a silent no-op: it answers a service call,
    // and the caller has to learn that nothing was switched.
    if (!contactCoilHandler_ || !contactCoilHandler_->isContactCoilHandlerEnabled()) {
        throw std::runtime_error("Safety controller not started; aux output not switched.");
    }

    contactCoilHandler_->setAuxOutput(index, state);
}

const std::unordered_map<RoverControllerGpio, bool> & RoverSafetyController::queryControlInterfaceIOStates()
{
    if (contactCoilHandler_ && contactCoilHandler_->isContactCoilHandlerEnabled()) {
        contactCoilHandler_->getIoState(io_state_cache_);
    }

    return io_state_cache_;
}

bool RoverSafetyController::isPinActive(const RoverControllerGpio pin)
{
    const auto & io_state = queryControlInterfaceIOStates();
    const auto it = io_state.find(pin);

    return it != io_state.end() && it->second;
}

SafetyLinkHealth RoverSafetyController::getHealth() const
{
    if (!contactCoilHandler_) {
        return SafetyLinkHealth {};
    }

    return contactCoilHandler_->getHealth();
}

}  // namespace rover_hardware_interface
