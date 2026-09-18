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

#ifndef ROVER_LED_APPLICATION_STOP_ANIMATION_USE_CASE_HPP_
#define ROVER_LED_APPLICATION_STOP_ANIMATION_USE_CASE_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "rover_led/application/led_types.hpp"
#include "rover_led/domain/ports/animation_catalog.hpp"

namespace rover_led
{

struct StopAnimationResult
{
    std::string name;
    // Segments the animation was playing (or queued) on, now stopped. Empty
    // when the animation wasn't playing anywhere.
    std::vector<std::string> stopped_segments;
};

// Stops an animation on every segment it covers, at its priority layer.
class StopAnimationUseCase
{

public:

    StopAnimationUseCase(std::shared_ptr<const IAnimationCatalog> catalog, SegmentMap segments);

    // Throws std::runtime_error on an unknown id or segment.
    StopAnimationResult execute(const std::size_t id);

private:

    std::shared_ptr<const IAnimationCatalog> catalog_;
    SegmentMap segments_;
};

}  // namespace rover_led

#endif  // ROVER_LED_APPLICATION_STOP_ANIMATION_USE_CASE_HPP_
