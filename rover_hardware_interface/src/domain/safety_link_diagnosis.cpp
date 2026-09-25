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

#include "rover_hardware_interface/domain/safety_link_diagnosis.hpp"

namespace rover_hardware_interface
{

SafetyLinkDiagnosis evaluateSafetyLinkHealth(
    const SafetyLinkHealth & health, const bool contactor_fault_latched,
    const bool contactor_failed_open)
{
    SafetyLinkDiagnosis result{SafetyLinkSeverity::kOk, "Safety PLC link healthy."};

    // Ordered least to most severe so the most serious condition owns the summary.
    if (contactor_failed_open) {
        result.severity = SafetyLinkSeverity::kWarn;
        result.message = "Motor contactor reports open while the E-Stop latch is clear - rover "
                         "will not drive.";
    }

    if (health.watchdog_miss_count > 0) {
        result.severity = SafetyLinkSeverity::kWarn;
        result.message = "Safety PLC heartbeat is landing late - the link is too slow for the "
                         "configured margin.";
    }

    if (!health.watchdog_running || !health.poll_running) {
        result.severity = SafetyLinkSeverity::kError;
        result.message = "A safety controller background thread is not running.";
    }

    if (contactor_fault_latched) {
        // The hazardous one: the stop was commanded and the contacts did not open.
        result.severity = SafetyLinkSeverity::kError;
        result.message = "E-Stop latch asserted but the motor contactor still reports engaged - "
                         "suspect welded contacts. Motion inhibited until sw_e_stop_latch_reset "
                         "and a hardware check.";
    }

    return result;
}

}  // namespace rover_hardware_interface
