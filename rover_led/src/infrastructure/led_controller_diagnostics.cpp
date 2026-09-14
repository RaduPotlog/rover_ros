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


#include "rover_led/infrastructure/led_controller_diagnostics.hpp"

#include <cmath>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace rover_led
{

namespace
{

const char * toString(const AnimationPriority priority)
{
    switch (priority) {
        case AnimationPriority::ERROR: return "ERROR";
        case AnimationPriority::ALERT: return "ALERT";
        case AnimationPriority::INFO: return "INFO";
        case AnimationPriority::STATE: return "STATE";
    }
    return "UNKNOWN";
}

std::string describeSegment(const LedSegmentSnapshot & segment)
{
    // Layers are ordered ERROR to STATE, so the first active one is what the segment shows.
    for (const auto & layer : segment.layers) {
        if (!layer.status) {
            continue;
        }

        const auto & status = *layer.status;
        std::string text = status.info.name + " [" + toString(layer.priority) + ", " +
            std::to_string(static_cast<int>(std::lround(status.progress * 100.0f))) + "%";
        if (status.repeating) {
            text += ", repeating";
        }
        if (status.queued > 0) {
            text += ", " + std::to_string(status.queued) + " queued";
        }
        return text + "]";
    }

    return "idle";
}

}  // namespace

void fillLedControllerStatus(
    const LedStateSnapshot & snapshot, const LedControllerDiagnostics & diagnostics,
    diagnostic_updater::DiagnosticStatusWrapper & status)
{
    using diagnostic_msgs::msg::DiagnosticStatus;

    status.add("Animations loaded", diagnostics.animations_loaded);
    status.add("Animations with unavailable type", diagnostics.unavailable_animations);
    status.add("Catalog warnings", diagnostics.catalog_warnings);
    status.add("Segment update failures (last tick)", diagnostics.segment_errors);
    status.add(
        "Last rejected animation request",
        diagnostics.last_rejected_request.value_or(std::string("none")));

    for (const auto & segment : snapshot.segments) {
        status.add(
            "Segment " + segment.name + " (channel " + std::to_string(segment.channel) + ")",
            describeSegment(segment));
    }

    if (diagnostics.render_error) {
        status.summary(DiagnosticStatus::ERROR, "Render failed: " + *diagnostics.render_error);
    } else if (diagnostics.segment_errors > 0) {
        status.summary(DiagnosticStatus::WARN, "Some segments failed to update their animation.");
    } else if (diagnostics.unavailable_animations > 0) {
        status.summary(DiagnosticStatus::WARN, "Some animations use an unavailable type.");
    } else {
        status.summary(DiagnosticStatus::OK, "Controller rendering.");
    }
}

}  // namespace rover_led
