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

#ifndef ROVER_LED_APPLICATION_LED_TYPES_HPP_
#define ROVER_LED_APPLICATION_LED_TYPES_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>

#include "rover_led/domain/led_components/led_panel.hpp"
#include "rover_led/domain/led_components/led_segment.hpp"

namespace rover_led
{

using SegmentMap = std::unordered_map<std::string, std::shared_ptr<LedSegment>>;
using PanelMap = std::unordered_map<std::size_t, std::shared_ptr<LedPanel>>;

}  // namespace rover_led

#endif  // ROVER_LED_APPLICATION_LED_TYPES_HPP_
