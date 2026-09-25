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

#ifndef ROVER_LED_APPLICATION_VALIDATE_ANIMATION_CATALOG_USE_CASE_HPP_
#define ROVER_LED_APPLICATION_VALIDATE_ANIMATION_CATALOG_USE_CASE_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "rover_led/domain/led_components/led_animation_description.hpp"
#include "rover_led/domain/ports/animation_factory.hpp"

namespace rover_led
{

struct UnavailableAnimationType
{
    std::size_t id;
    std::string name;
    std::string type;
};

struct ValidateAnimationCatalogResult
{
    // Catalog animations with at least one unavailable type; they can't be displayed.
    std::size_t unavailable_animations = 0;
    // One entry per animation whose type the factory can't create, in catalog order.
    std::vector<UnavailableAnimationType> unavailable_types;
};

// Checks that the factory can create every animation type the catalog uses.
class ValidateAnimationCatalogUseCase
{

public:

    explicit ValidateAnimationCatalogUseCase(std::shared_ptr<IAnimationFactory> factory);

    // Creates (and discards) one animation per entry of every catalog animation. Only
    // std::runtime_error marks a type unavailable; anything else propagates.
    ValidateAnimationCatalogResult execute(const std::vector<LedAnimationDescription> & animations);

private:

    std::shared_ptr<IAnimationFactory> factory_;
};

}  // namespace rover_led

#endif  // ROVER_LED_APPLICATION_VALIDATE_ANIMATION_CATALOG_USE_CASE_HPP_
