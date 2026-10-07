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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_IMU_DATA_HEALTH_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_IMU_DATA_HEALTH_HPP_

#include <atomic>
#include <cstdint>
#include <string>

#include "rover_hardware_interface/domain/health_verdict.hpp"

namespace rover_hardware_interface
{

// What the monitor saw of the imu/data stream. Taken as a snapshot by diagnostics.
struct ImuDataStats
{
    bool received_any{false};
    bool last_finite{false};          // Every field of the latest message was finite (no NaN/inf).
    double last_message_age_s{-1.0};  // Time since the latest message, -1 if none yet.
    double last_valid_age_s{-1.0};    // Time since the latest all-finite message, -1 if none yet.
    double since_start_s{0.0};        // Time the monitor has been running.
    std::uint64_t total{0};
    std::uint64_t valid{0};
    std::uint64_t not_finite{0};
};

// Lock-free recorder for the imu/data stream, called from a subscription callback and read by the
// diagnostics thread. Relaxed atomics; timestamps are caller-supplied steady-clock nanoseconds so
// the class has no clock of its own and is trivially testable. One writer thread.
class ImuDataRecorder
{

public:

    void onMessage(const bool all_finite, const std::int64_t now_ns)
    {
        total_.fetch_add(1, std::memory_order_relaxed);
        last_rx_ns_.store(now_ns, std::memory_order_relaxed);
        last_finite_.store(all_finite, std::memory_order_relaxed);

        if (all_finite) {
            valid_.fetch_add(1, std::memory_order_relaxed);
            last_valid_ns_.store(now_ns, std::memory_order_relaxed);
        } else {
            not_finite_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    ImuDataStats snapshot(const std::int64_t now_ns, const std::int64_t start_ns) const
    {
        ImuDataStats s;
        s.total = total_.load(std::memory_order_relaxed);
        s.valid = valid_.load(std::memory_order_relaxed);
        s.not_finite = not_finite_.load(std::memory_order_relaxed);
        s.received_any = s.total > 0;
        s.last_finite = last_finite_.load(std::memory_order_relaxed);
        s.since_start_s = static_cast<double>(now_ns - start_ns) * 1e-9;

        if (s.received_any) {
            s.last_message_age_s =
                static_cast<double>(now_ns - last_rx_ns_.load(std::memory_order_relaxed)) * 1e-9;
        }

        if (s.valid > 0) {
            s.last_valid_age_s =
                static_cast<double>(now_ns - last_valid_ns_.load(std::memory_order_relaxed)) * 1e-9;
        }

        return s;
    }

private:

    std::atomic<std::uint64_t> total_{0};
    std::atomic<std::uint64_t> valid_{0};
    std::atomic<std::uint64_t> not_finite_{0};
    std::atomic<std::int64_t> last_rx_ns_{0};
    std::atomic<std::int64_t> last_valid_ns_{0};
    std::atomic<bool> last_finite_{false};
};

struct ImuDataThresholds
{
    double startup_grace_s{20.0};  // Time allowed after start before "nothing yet" is an error.
    double stale_s{1.0};           // Longest acceptable gap between messages (25 Hz nominal).
};

// Whatever the cause - the hardware component never activated, the device detached, the
// broadcaster stopped - the symptom downstream is the same, so this judges the data, not the
// component: valid data is OK, anything else is not.
inline HealthVerdict evaluateImuDataHealth(const ImuDataStats & s, const ImuDataThresholds & t = {})
{
    if (!s.received_any) {
        if (s.since_start_s < t.startup_grace_s) {
            return {HealthLevel::kWarn, "Waiting for the first imu/data message."};
        }

        return {HealthLevel::kError,
                "No imu/data received since startup: the IMU broadcaster is not publishing."};
    }

    if (s.last_message_age_s > t.stale_s) {
        return {HealthLevel::kError,
                "imu/data is stale: last message " +
                    std::to_string(static_cast<std::int64_t>(s.last_message_age_s)) + " s ago."};
    }

    if (!s.last_finite) {
        return {HealthLevel::kError,
                "imu/data contains NaN: the IMU hardware component is not active or the device is "
                "detached."};
    }

    return {HealthLevel::kOk, "IMU data is valid."};
}

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_IMU_DATA_HEALTH_HPP_
