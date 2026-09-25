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

#include "rover_crsf_teleop/domain/safety_io_monitor.hpp"

namespace rover_crsf_teleop
{

SafetyIoMonitor::SafetyIoMonitor(const std::chrono::milliseconds timeout)
: timeout_(timeout)
{
}

void SafetyIoMonitor::onSample(const SteadyTime now, const SafetyIoFlags & flags)
{
    last_ = Sample{flags, now};
}

void SafetyIoMonitor::clear()
{
    last_.reset();
}

EStopState SafetyIoMonitor::eStopState(const SteadyTime now) const
{
    // Anything older than the timeout is "cannot verify" rather than "still whatever it was".
    // Strict >, as LinkMonitor: a sample exactly the timeout old is still fresh.
    if (!last_ || (now - last_->received_at) > timeout_) {
        return EStopState::kUnknown;
    }

    return isSafeToCalibrate(last_->flags) ? EStopState::kEngaged : EStopState::kReleased;
}

std::optional<std::chrono::milliseconds> SafetyIoMonitor::sampleAge(const SteadyTime now) const
{
    if (!last_) {
        return std::nullopt;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - last_->received_at);
}

}  // namespace rover_crsf_teleop
