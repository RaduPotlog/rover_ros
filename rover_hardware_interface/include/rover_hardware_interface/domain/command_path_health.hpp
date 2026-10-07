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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_COMMAND_PATH_HEALTH_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_COMMAND_PATH_HEALTH_HPP_

#include <cstdint>
#include <string>

#include "rover_hardware_interface/domain/health_verdict.hpp"

namespace rover_hardware_interface
{

// Tells whether a cumulative, only-ever-growing counter is still growing. A diagnostics task
// calls update() once per cycle with the counter's current total, so a total that stopped
// growing (an error that happened and went away) reads differently from one that is still
// happening. Single-threaded: one diagnostics thread owns each instance.
class ErrorTrend
{

public:

    // `now_ns` is a steady-clock time in nanoseconds. The first call counts a non-zero total as an
    // increase, since errors that predate the first look are still news.
    void update(const std::uint64_t total, const std::int64_t now_ns)
    {
        if (total > total_) {
            increased_ = true;
            last_increase_ns_ = now_ns;
        }

        total_ = total;
    }

    // An increase was seen no more than `window_ns` ago.
    bool increasedWithin(const std::int64_t now_ns, const std::int64_t window_ns) const
    {
        return increased_ && (now_ns - last_increase_ns_) <= window_ns;
    }

    // Seconds since the counter last grew, or -1.0 if it never has.
    double secondsSinceIncrease(const std::int64_t now_ns) const
    {
        return increased_ ? static_cast<double>(now_ns - last_increase_ns_) * 1e-9 : -1.0;
    }

    std::uint64_t total() const { return total_; }

private:

    std::uint64_t total_{0};
    std::int64_t last_increase_ns_{0};
    bool increased_{false};
};

struct CommandPathHealthInput
{
    bool command_errors_increasing{false};   // Driver completions still returning errors now.
    std::uint64_t command_errors_total{0};   // Cumulative completions that returned an error.
    double seconds_since_command_error{-1.0};  // -1 if none.
    std::uint64_t dropped_no_driver{0};
    std::uint64_t write_exceptions{0};
};

// ERROR only while commands are being lost *right now*; a one-off from the past is a WARN, and
// dropping a command because the previous one is still in flight is ordinary back-pressure and is
// deliberately not an input here (it is reported as a ratio, not a fault).
inline HealthVerdict evaluateCommandPathHealth(const CommandPathHealthInput & in)
{
    if (in.command_errors_increasing) {
        return {HealthLevel::kError,
                "Wheel commands are failing: the driver is still returning errors (the board may be "
                "detached)."};
    }

    if (in.command_errors_total > 0) {
        return {HealthLevel::kWarn,
                "Earlier command errors, none recent (last " +
                    std::to_string(static_cast<std::int64_t>(in.seconds_since_command_error)) + " s ago)."};
    }

    if (in.dropped_no_driver > 0) {
        return {HealthLevel::kWarn, "Commands lost: a driver object was missing."};
    }

    if (in.write_exceptions > 0) {
        return {HealthLevel::kWarn, "Commands lost: a write operation threw."};
    }

    return {HealthLevel::kOk, "Command path counters."};
}

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_COMMAND_PATH_HEALTH_HPP_
