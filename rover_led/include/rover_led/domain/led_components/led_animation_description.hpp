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

#ifndef ROVER_LED_DOMAIN_LED_COMPONENTS_LED_ANIMATION_DESCRIPTION_HPP_
#define ROVER_LED_DOMAIN_LED_COMPONENTS_LED_ANIMATION_DESCRIPTION_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace rover_led
{

// One animation plugin applied to a group of segments.
struct AnimationDescription
{
    std::string type;
    std::vector<std::string> segments;
    YAML::Node animation;
};

// A catalog entry: what "animation id N" means.
struct LedAnimationDescription
{
    std::size_t id;
    std::uint8_t priority;
    std::string name;
    float timeout;
    std::vector<AnimationDescription> animations;

    static constexpr std::uint8_t kDefaultPriority = 3;
    static constexpr float kDefaultTimeout = 120.0f;
    static constexpr std::array<std::uint8_t, 4> kValidPriorities = {0, 1, 2, 3};
};

// A request to show a catalog animation.
struct LedAnimationRequest
{
    std::size_t id;
    std::string param;
    bool repeating = false;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_LED_COMPONENTS_LED_ANIMATION_DESCRIPTION_HPP_
