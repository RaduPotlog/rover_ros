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

#include "rover_hardware_interface/domain/contactor_monitor.hpp"

namespace rover_hardware_interface
{

ContactorMonitor::ContactorMonitor(const std::chrono::milliseconds drop_out_tolerance)
: drop_out_tolerance_(drop_out_tolerance)
{

}

ContactorFault ContactorMonitor::update(
    const bool latch_active, const bool contactor_engaged,
    const std::chrono::steady_clock::time_point now)
{
    // The latch being asserted means "the relay has commanded the contactor open". Disagreement
    // is that command not being reflected by the contacts.
    const bool disagrees = latch_active && contactor_engaged;

    if (disagrees) {
        if (!disagreement_active_) {
            disagreement_active_ = true;
            disagreement_since_ = now;
        }

        if ((now - disagreement_since_) >= drop_out_tolerance_) {
            welded_fault_latched_ = true;
        }
    } else {
        disagreement_active_ = false;
    }

    failed_open_ = (!latch_active && !contactor_engaged);

    if (welded_fault_latched_.load()) {
        return ContactorFault::kWeldedSuspected;
    }

    if (failed_open_.load()) {
        return ContactorFault::kFailedOpen;
    }

    return ContactorFault::kNone;
}

bool ContactorMonitor::isWeldedFaultLatched() const
{
    return welded_fault_latched_.load();
}

bool ContactorMonitor::isFailedOpen() const
{
    return failed_open_.load();
}

std::chrono::milliseconds ContactorMonitor::disagreementDuration(
    const std::chrono::steady_clock::time_point now) const
{
    if (!disagreement_active_) {
        return std::chrono::milliseconds(0);
    }

    return std::chrono::duration_cast<std::chrono::milliseconds>(now - disagreement_since_);
}

void ContactorMonitor::reset()
{
    welded_fault_latched_ = false;
    failed_open_ = false;
    disagreement_active_ = false;
}

}  // namespace rover_hardware_interface
