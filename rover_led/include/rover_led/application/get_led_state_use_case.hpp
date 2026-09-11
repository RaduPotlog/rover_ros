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

#ifndef ROVER_LED_APPLICATION_GET_LED_STATE_USE_CASE_HPP_
#define ROVER_LED_APPLICATION_GET_LED_STATE_USE_CASE_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "rover_led/application/led_types.hpp"
#include "rover_led/domain/led_components/led_segment.hpp"
#include "rover_led/domain/led_components/segment_layer_interface.hpp"

namespace rover_led
{

struct LedLayerSnapshot
{
    AnimationPriority priority;
    // Empty when nothing plays on the layer.
    std::optional<LayerStatus> status;
};

struct LedSegmentSnapshot
{
    std::string name;
    std::size_t channel;
    // Ordered ERROR to STATE.
    std::vector<LedLayerSnapshot> layers;
};

struct LedStateSnapshot
{
    // Sorted by segment name.
    std::vector<LedSegmentSnapshot> segments;
};

// Reports which animation plays on every layer of every segment.
class GetLedStateUseCase
{

public:

    explicit GetLedStateUseCase(SegmentMap segments);

    LedStateSnapshot execute() const;

private:

    SegmentMap segments_;
};

}  // namespace rover_led

#endif  // ROVER_LED_APPLICATION_GET_LED_STATE_USE_CASE_HPP_
