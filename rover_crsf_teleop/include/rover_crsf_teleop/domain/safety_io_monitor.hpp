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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_SAFETY_IO_MONITOR_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_SAFETY_IO_MONITOR_HPP_

#include <chrono>
#include <optional>

#include "rover_crsf_teleop/domain/link_monitor.hpp"
#include "rover_crsf_teleop/domain/safety_io_flags.hpp"

namespace rover_crsf_teleop
{

// What the node can say about the rover's E-Stop from the safety IO it last received - the
// same "a stale sample can't be trusted" rule LinkMonitor applies to the RC link.
//   no sample (never received, or clear()ed)  -> kUnknown
//   sample older than timeout (strict >)      -> kUnknown   (never its last value)
//   fresh, isSafeToCalibrate(flags)           -> kEngaged
//   fresh, otherwise                          -> kReleased
// Aged from ARRIVAL (the `now` passed to onSample), not SafetyStatus.io_sample_time: SafetyStatus
// has no header, the two differ by at most one poll period, and arrival is what detects a
// publisher that has stopped. Fails closed: only a fresh permitting sample is kEngaged.
class SafetyIoMonitor
{

public:

    explicit SafetyIoMonitor(std::chrono::milliseconds timeout);

    void onSample(SteadyTime now, const SafetyIoFlags & flags);

    // Forget the sample: its link reported unhealthy (fields are last-known-good), or cleanup.
    void clear();

    EStopState eStopState(SteadyTime now) const;

    // duration_cast<milliseconds>(now - arrival) while a sample is held (stale included); nullopt
    // otherwise.
    std::optional<std::chrono::milliseconds> sampleAge(SteadyTime now) const;

private:

    struct Sample
    {
        SafetyIoFlags flags;
        SteadyTime received_at;
    };

    std::chrono::milliseconds timeout_;
    std::optional<Sample> last_;
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_SAFETY_IO_MONITOR_HPP_
