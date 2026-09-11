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

#ifndef ROVER_LED_APPLICATION_SET_ANIMATION_USE_CASE_HPP_
#define ROVER_LED_APPLICATION_SET_ANIMATION_USE_CASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rover_led/application/led_types.hpp"
#include "rover_led/domain/led_components/led_animation_description.hpp"
#include "rover_led/domain/ports/animation_catalog.hpp"
#include "rover_led/domain/ports/animation_factory.hpp"

namespace rover_led
{

struct SetAnimationResult
{
    std::string name;
    // Segments whose priority layer rejected the animation (full ALERT queue).
    std::vector<std::string> rejected_segments;
};

// Looks a catalog animation up and puts it on its segments. All animations
// are created and initialized before any segment is touched, so a failure
// leaves the currently displayed animations unchanged.
class SetAnimationUseCase
{

public:

    SetAnimationUseCase(
        std::shared_ptr<const IAnimationCatalog> catalog,
        std::shared_ptr<IAnimationFactory> factory,
        SegmentMap segments,
        const float controller_frequency);

    // Throws std::runtime_error on an unknown id or segment, or when an
    // animation fails to load or initialize.
    SetAnimationResult execute(const LedAnimationRequest & request);

private:

    std::shared_ptr<const IAnimationCatalog> catalog_;
    std::shared_ptr<IAnimationFactory> factory_;
    SegmentMap segments_;
    const float controller_frequency_;
};

}  // namespace rover_led

#endif  // ROVER_LED_APPLICATION_SET_ANIMATION_USE_CASE_HPP_
