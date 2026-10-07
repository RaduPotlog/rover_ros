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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_COMMAND_STATS_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_COMMAND_STATS_HPP_

#include <atomic>
#include <cstdint>
#include <limits>

namespace rover_hardware_interface
{

// How a motor driver's asynchronous velocity command completed.
enum class CommandCompletion
{
    kOk,
    kFailsafe,  // The board's hardware watchdog rejected it (also latched elsewhere as a fault).
    kError,     // Any other non-OK return code - the command may never have reached the motor.
};

// Cumulative counters about one wheel's command path, from sendCmdVel() down to the completion of
// the driver's asynchronous call. Instrumentation only: nothing in the control loop reads these.
// "Dropped" commands are the ones sendCmdVel() silently never issued.
struct MotorCommandStats
{
    std::uint64_t submitted{0};           // Async velocity calls actually issued to the driver.
    std::uint64_t dropped_pending{0};     // Skipped: the previous async call had not completed.
    std::uint64_t dropped_no_driver{0};   // Skipped: the owning driver object no longer exists.
    std::uint64_t completed_ok{0};
    std::uint64_t completed_failsafe{0};
    std::uint64_t completed_error{0};     // Completed with a non-OK, non-failsafe return code.
    std::int32_t last_error_code{0};      // Last non-OK return code seen (0 = none yet).
    std::uint64_t latency_sum_us{0};      // Sum of submit -> completion times of all completions.
    std::uint32_t latency_last_us{0};
    std::uint32_t latency_max_us{0};
    bool pending{false};                  // An async call is in flight right now.
    std::uint32_t pending_age_ms{0};      // How long that call has been in flight (0 if none).

    std::uint64_t completed() const { return completed_ok + completed_failsafe + completed_error; }

    // Everything sendCmdVel() was asked to send, whether or not it went out.
    std::uint64_t attempted() const { return submitted + dropped_pending + dropped_no_driver; }

    double meanLatencyUs() const
    {
        const auto n = completed();
        return n == 0 ? 0.0 : static_cast<double>(latency_sum_us) / static_cast<double>(n);
    }

    double droppedPendingRatio() const
    {
        const auto n = attempted();
        return n == 0 ? 0.0 : static_cast<double>(dropped_pending) / static_cast<double>(n);
    }
};

// Lock-free, allocation-free recorder for MotorCommandStats, safe to call from the RT thread
// (onSubmit / onDropped*) and from the SDK's completion-callback thread (onCompletion) at once.
// Every counter is a relaxed atomic: a diagnostics read may see a few counters from slightly
// different instants, which is fine for statistics. Timestamps are caller-supplied nanoseconds
// (steady clock) so the class stays free of clocks and is trivially testable.
//
// Threading contract: onSubmit/onDroppedPending/onDroppedNoDriver from one thread, onCompletion
// from one (other) thread. The latency maximum is a plain read-compare-write for that reason.
class CommandStatsRecorder
{

public:

    void onSubmit(const std::int64_t now_ns)
    {
        submit_ns_.store(now_ns, std::memory_order_relaxed);
        submitted_.fetch_add(1, std::memory_order_relaxed);
    }

    void onDroppedPending() { dropped_pending_.fetch_add(1, std::memory_order_relaxed); }

    void onDroppedNoDriver() { dropped_no_driver_.fetch_add(1, std::memory_order_relaxed); }

    // `code` is the SDK return code (stored only for non-OK results).
    void onCompletion(const CommandCompletion result, const std::int32_t code, const std::int64_t now_ns)
    {
        std::int64_t elapsed_ns = now_ns - submit_ns_.load(std::memory_order_relaxed);

        if (elapsed_ns < 0) {
            elapsed_ns = 0;
        }

        std::int64_t elapsed_us = elapsed_ns / 1000;

        if (elapsed_us > std::numeric_limits<std::uint32_t>::max()) {
            elapsed_us = std::numeric_limits<std::uint32_t>::max();
        }

        const auto latency_us = static_cast<std::uint32_t>(elapsed_us);

        latency_sum_us_.fetch_add(latency_us, std::memory_order_relaxed);
        latency_last_us_.store(latency_us, std::memory_order_relaxed);

        if (latency_us > latency_max_us_.load(std::memory_order_relaxed)) {
            latency_max_us_.store(latency_us, std::memory_order_relaxed);
        }

        switch (result) {
            case CommandCompletion::kOk:
                completed_ok_.fetch_add(1, std::memory_order_relaxed);
                break;

            case CommandCompletion::kFailsafe:
                completed_failsafe_.fetch_add(1, std::memory_order_relaxed);
                last_error_code_.store(code, std::memory_order_relaxed);
                break;

            case CommandCompletion::kError:
                completed_error_.fetch_add(1, std::memory_order_relaxed);
                last_error_code_.store(code, std::memory_order_relaxed);
                break;
        }
    }

    // `pending` is whether an async call is in flight; `now_ns` ages it against the last submit.
    MotorCommandStats snapshot(const bool pending, const std::int64_t now_ns) const
    {
        MotorCommandStats s;
        s.submitted = submitted_.load(std::memory_order_relaxed);
        s.dropped_pending = dropped_pending_.load(std::memory_order_relaxed);
        s.dropped_no_driver = dropped_no_driver_.load(std::memory_order_relaxed);
        s.completed_ok = completed_ok_.load(std::memory_order_relaxed);
        s.completed_failsafe = completed_failsafe_.load(std::memory_order_relaxed);
        s.completed_error = completed_error_.load(std::memory_order_relaxed);
        s.last_error_code = last_error_code_.load(std::memory_order_relaxed);
        s.latency_sum_us = latency_sum_us_.load(std::memory_order_relaxed);
        s.latency_last_us = latency_last_us_.load(std::memory_order_relaxed);
        s.latency_max_us = latency_max_us_.load(std::memory_order_relaxed);
        s.pending = pending;

        if (pending) {
            const std::int64_t age_ms = (now_ns - submit_ns_.load(std::memory_order_relaxed)) / 1000000;

            if (age_ms > 0) {
                s.pending_age_ms = age_ms > std::numeric_limits<std::uint32_t>::max()
                                       ? std::numeric_limits<std::uint32_t>::max()
                                       : static_cast<std::uint32_t>(age_ms);
            }
        }

        return s;
    }

private:

    std::atomic<std::int64_t> submit_ns_{0};
    std::atomic<std::uint64_t> submitted_{0};
    std::atomic<std::uint64_t> dropped_pending_{0};
    std::atomic<std::uint64_t> dropped_no_driver_{0};
    std::atomic<std::uint64_t> completed_ok_{0};
    std::atomic<std::uint64_t> completed_failsafe_{0};
    std::atomic<std::uint64_t> completed_error_{0};
    std::atomic<std::int32_t> last_error_code_{0};
    std::atomic<std::uint64_t> latency_sum_us_{0};
    std::atomic<std::uint32_t> latency_last_us_{0};
    std::atomic<std::uint32_t> latency_max_us_{0};
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_COMMAND_STATS_HPP_
