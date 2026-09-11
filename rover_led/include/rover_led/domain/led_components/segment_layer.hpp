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

#ifndef ROVER_LED_DOMAIN_LED_COMPONENTS_SEGMENT_LAYER_HPP_
#define ROVER_LED_DOMAIN_LED_COMPONENTS_SEGMENT_LAYER_HPP_

#include <cstddef>
#include <memory>

#include "rover_led/domain/led_components/segment_layer_interface.hpp"

namespace rover_led
{

// Replaces the current animation; optionally repeats it forever.
class SegmentLayer : public SegmentLayerInterface
{

public:

    SegmentLayer(const std::size_t num_led, const bool invert_led_order);

    bool setAnimation(
        const std::shared_ptr<Animation> & animation,
        const bool repeating) override;

    void updateAnimation() override;

protected:

    bool repeating_ = false;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_LED_COMPONENTS_SEGMENT_LAYER_HPP_
