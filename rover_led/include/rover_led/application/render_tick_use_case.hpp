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

#ifndef ROVER_LED_APPLICATION_RENDER_TICK_USE_CASE_HPP_
#define ROVER_LED_APPLICATION_RENDER_TICK_USE_CASE_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "rover_led/application/led_types.hpp"
#include "rover_led/domain/led_components/segment_converter.hpp"

namespace rover_led
{

struct RenderTickResult
{
    // "<segment>: <reason>" for every segment whose animation failed to
    // advance; the other segments are still rendered.
    std::vector<std::string> segment_errors;

    // Set when the segments could not be composed into panel frames; no
    // frames are produced in that case.
    std::optional<std::string> error;

    // RGBA8 frame per panel channel.
    std::map<std::size_t, std::vector<std::uint8_t>> frames;
};

// One controller cycle: advance every segment animation, then compose the
// segments into panel frames.
class RenderTickUseCase
{

public:

    RenderTickUseCase(SegmentMap segments, PanelMap panels);

    RenderTickResult execute();

private:

    SegmentMap segments_;
    PanelMap panels_;
    SegmentConverter segment_converter_;
};

}  // namespace rover_led

#endif  // ROVER_LED_APPLICATION_RENDER_TICK_USE_CASE_HPP_
