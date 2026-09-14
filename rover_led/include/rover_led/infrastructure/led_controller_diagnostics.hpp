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


#ifndef ROVER_LED_INFRASTRUCTURE_LED_CONTROLLER_DIAGNOSTICS_HPP_
#define ROVER_LED_INFRASTRUCTURE_LED_CONTROLLER_DIAGNOSTICS_HPP_

#include <cstddef>
#include <optional>
#include <string>

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

#include "rover_led/application/get_led_state_use_case.hpp"

namespace rover_led
{

// What LedControllerNode recorded about its own operation, for the "Led controller status" task.
struct LedControllerDiagnostics
{
    std::size_t animations_loaded = 0;
    // Catalog animations referencing a plugin type that could not be loaded (config problem).
    std::size_t unavailable_animations = 0;
    std::size_t catalog_warnings = 0;
    // Set while the latest render tick could not compose panel frames; cleared on success.
    std::optional<std::string> render_error;
    // Segments whose animation failed to advance on the latest render tick.
    std::size_t segment_errors = 0;
    // Last led/set_animation request that failed or was (partially) dropped. Informational only:
    // it does not grade the status, so one bad request can't hold a WARN forever.
    std::optional<std::string> last_rejected_request;
};

// Levels: ERROR while the latest render tick failed; WARN on segment update failures or
// animations with unavailable types; OK otherwise. One value per segment names the animation on
// its highest-priority active layer.
void fillLedControllerStatus(
    const LedStateSnapshot & snapshot, const LedControllerDiagnostics & diagnostics,
    diagnostic_updater::DiagnosticStatusWrapper & status);

}  // namespace rover_led

#endif  // ROVER_LED_INFRASTRUCTURE_LED_CONTROLLER_DIAGNOSTICS_HPP_
