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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_CONTACTOR_MONITOR_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_CONTACTOR_MONITOR_HPP_

#include <atomic>
#include <chrono>

namespace rover_hardware_interface
{

// Default grace period before latch/contactor disagreement is called a fault. Must cover the
// contactor's physical drop-out time plus one safety-IO poll period plus scheduling jitter -
// otherwise every ordinary E-Stop would raise a welded-contactor fault during the moments
// between the relay latching and the contacts actually parting. 500 ms is generous against a
// contactor that drops out in tens of milliseconds and a 100 ms poll.
constexpr unsigned kDefaultContactorDropOutToleranceMs = 500;

enum class ContactorFault
{
    kNone,

    // The E-Stop latch is asserted but the contactor still reports engaged, for longer than the
    // drop-out tolerance. This is the hazardous case: the stop was commanded and the motors may
    // still be live - classically a welded contactor, but any failure that leaves the contacts
    // closed presents identically. Latched; only an explicit reset clears it.
    kWeldedSuspected,

    // The latch is clear but the contactor reports open. Not hazardous - the rover simply will
    // not drive - but it explains an otherwise mysterious dead machine, so it is reported.
    // Not latched.
    kFailedOpen,
};

// Cross-checks the E-Stop latch against the contactor's auxiliary-contact feedback - the software
// half of an EDM (external device monitoring) loop.
//
// The rover's safety relay already routes the contactor's aux contact back as a readable input
// (GPIO_MOTOR_CONTACTOR_ENGAGED / COIL_0), so the signal needed to catch a contactor that failed
// to open has always been on the wire; nothing consumed it. Without this check a welded contactor
// is invisible to software: the latch reads asserted, the rover is believed stopped, and the
// motors are still powered.
//
// Pure domain logic - `now` is injected rather than read from a clock, so the behaviour is
// testable without waiting in real time.
class ContactorMonitor
{

public:

    explicit ContactorMonitor(
        const std::chrono::milliseconds drop_out_tolerance =
            std::chrono::milliseconds(kDefaultContactorDropOutToleranceMs));

    // Feeds one observation and returns this cycle's verdict. Call every read() cycle.
    ContactorFault update(
        const bool latch_active, const bool contactor_engaged,
        const std::chrono::steady_clock::time_point now);

    // True once kWeldedSuspected has been reached, until reset(). Latched deliberately: a
    // contactor that failed to open once must not be allowed to look healthy again just because
    // the latch was subsequently cleared.
    bool isWeldedFaultLatched() const;

    // Momentary view of the last update() - not latched.
    bool isFailedOpen() const;

    // How long latch/contactor disagreement has persisted, for diagnostics. Zero when they agree.
    std::chrono::milliseconds disagreementDuration(
        const std::chrono::steady_clock::time_point now) const;

    // Clears the latched welded fault. Wired to the same operator action that clears the relay
    // latch, so a fault never silently disappears on its own.
    void reset();

private:

    const std::chrono::milliseconds drop_out_tolerance_;

    // Guarded by nothing: only ever touched from the single read() cycle. The two published
    // flags are atomic because diagnostics reads them from another thread.
    bool disagreement_active_ = false;
    std::chrono::steady_clock::time_point disagreement_since_ {};

    std::atomic_bool welded_fault_latched_ {false};
    std::atomic_bool failed_open_ {false};
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_CONTACTOR_MONITOR_HPP_
