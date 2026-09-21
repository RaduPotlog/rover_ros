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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_SAFETY_LINK_HEALTH_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_SAFETY_LINK_HEALTH_HPP_

#include <cstdint>

namespace rover_hardware_interface
{

// Health of the link to the safety controller and of the background threads that service it.
//
// Lives in domain/ (as a plain value, no behaviour) because RoverGpioPort exposes it and domain
// code may not include infrastructure headers - the same reason RoverControllerGpio moved here.
// The concrete Modbus-backed implementation fills it in; diagnostics reads it.
//
// This exists because PLC link health previously reached nothing: read()/write() always return
// return_type::OK, so the resource manager never learns the safety link is down, and there was no
// diagnostic named for it either. A stalled heartbeat is caught by the relay, which is correct -
// but "the relay stopped us" is a much worse thing to read in a log than "the safety link went
// quiet 3 s ago".
struct SafetyLinkHealth
{
    static constexpr uint64_t kUnknownAgeMs = UINT64_MAX;

    bool watchdog_running = false;
    bool poll_running = false;

    // Time since the last *successful* heartbeat write / IO poll. kUnknownAgeMs before the first
    // success. The heartbeat age is the one to compare against the relay's watchdog window.
    uint64_t last_kick_age_ms = kUnknownAgeMs;
    uint64_t last_poll_age_ms = kUnknownAgeMs;

    // Heartbeat ticks that landed later than their own deadline. Non-zero and rising means the
    // safety link is too slow for the configured margin - the early warning for exactly the
    // nuisance-trip failure the split-thread design was introduced to prevent.
    uint64_t watchdog_miss_count = 0;
    uint64_t watchdog_error_count = 0;
    uint64_t poll_error_count = 0;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_SAFETY_LINK_HEALTH_HPP_
