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

#include "rover_led/application/validate_animation_catalog_use_case.hpp"

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rover_led
{

ValidateAnimationCatalogUseCase::ValidateAnimationCatalogUseCase(std::shared_ptr<IAnimationFactory> factory)
: factory_(std::move(factory))
{

}

ValidateAnimationCatalogResult ValidateAnimationCatalogUseCase::execute(
    const std::vector<LedAnimationDescription> & animations)
{
    ValidateAnimationCatalogResult result;

    for (const auto & led_animation : animations) {
        bool available = true;

        for (const auto & animation : led_animation.animations) {
            try {
                factory_->create(animation.type);
            } catch (const std::runtime_error &) {
                available = false;
                result.unavailable_types.push_back({led_animation.id, led_animation.name, animation.type});
            }
        }

        if (!available) {
            ++result.unavailable_animations;
        }
    }

    return result;
}

}  // namespace rover_led
