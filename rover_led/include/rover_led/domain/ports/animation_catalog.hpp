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

#ifndef ROVER_LED_DOMAIN_PORTS_ANIMATION_CATALOG_HPP_
#define ROVER_LED_DOMAIN_PORTS_ANIMATION_CATALOG_HPP_

#include <cstddef>
#include <optional>

#include "rover_led/domain/led_components/led_animation_description.hpp"

namespace rover_led
{

// The set of animations the robot knows, looked up by id.
class IAnimationCatalog
{

public:

    virtual ~IAnimationCatalog() = default;

    virtual std::optional<LedAnimationDescription> find(const std::size_t id) const = 0;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_PORTS_ANIMATION_CATALOG_HPP_
