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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_SAFETY_LINK_DIAGNOSIS_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_SAFETY_LINK_DIAGNOSIS_HPP_

#include <string>

#include "rover_hardware_interface/domain/safety_link_health.hpp"

namespace rover_hardware_interface
{

// Severity of the "safety plc link" diagnostic, free of diagnostic_updater so the domain can
// decide it. RoverSystem maps it onto the DiagnosticStatus level.
enum class SafetyLinkSeverity
{
    kOk,
    kWarn,
    kError,
};

struct SafetyLinkDiagnosis
{
    SafetyLinkSeverity severity;
    std::string message;
};

// Summary of the "safety plc link" diagnostic. Priority (highest wins): latched contactor
// fault > a stopped safety thread > late heartbeat > contactor failed open > healthy.
// Error counters and ages are deliberately NOT inputs to the verdict (today's behaviour).
SafetyLinkDiagnosis evaluateSafetyLinkHealth(
    const SafetyLinkHealth & health, const bool contactor_fault_latched,
    const bool contactor_failed_open);

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_SAFETY_LINK_DIAGNOSIS_HPP_
