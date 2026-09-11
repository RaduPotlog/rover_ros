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

#ifndef ROVER_LED_DOMAIN_PORTS_ANIMATION_FACTORY_HPP_
#define ROVER_LED_DOMAIN_PORTS_ANIMATION_FACTORY_HPP_

#include <memory>
#include <string>

#include "rover_led/domain/animation/animation.hpp"

namespace rover_led
{

// Creates an uninitialized animation from its type name
// (e.g. "rover_led::ImageAnimation"). Throws std::runtime_error if the type
// is unknown.
class IAnimationFactory
{

public:

    virtual ~IAnimationFactory() = default;

    virtual std::shared_ptr<Animation> create(const std::string & type) = 0;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_PORTS_ANIMATION_FACTORY_HPP_
