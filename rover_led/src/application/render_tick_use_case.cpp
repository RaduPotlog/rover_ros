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

#include "rover_led/application/render_tick_use_case.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace rover_led
{

RenderTickUseCase::RenderTickUseCase(SegmentMap segments, PanelMap panels)
: segments_(std::move(segments))
, panels_(std::move(panels))
{

}

RenderTickResult RenderTickUseCase::execute()
{
    RenderTickResult result;

    for (auto & [segment_name, segment] : segments_) {
        try {
            if (segment->hasAnimation()) {
                segment->updateAnimation();
            }
        } catch (const std::runtime_error & e) {
            result.segment_errors.push_back(segment_name + ": " + e.what());
        }
    }

    try {
        segment_converter_.convert(segments_, panels_);
    } catch (const std::runtime_error & e) {
        result.error = e.what();
        return result;
    }

    for (const auto & [channel, panel] : panels_) {
        result.frames.emplace(channel, panel->getFrame());
    }

    return result;
}

}  // namespace rover_led
