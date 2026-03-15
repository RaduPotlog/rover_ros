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

#ifndef ROVER_LED_LED_COMPONENTS_LED_PANEL_HPP_
#define ROVER_LED_LED_COMPONENTS_LED_PANEL_HPP_

#include <cstdint>
#include <vector>

namespace rover_led
{

class LedPanel
{

public:
    
    LedPanel(const std::size_t num_led);

    ~LedPanel() = default;

    void updateFrame(const std::size_t iterator_first, const std::vector<std::uint8_t> & values);

    std::vector<std::uint8_t> getFrame() const 
    { 
        return frame_; 
    }
    
    std::size_t getNumberOfLeds() const 
    { 
        return num_led_; 
    }

private:

    const std::size_t num_led_;
    std::vector<std::uint8_t> frame_;
};

}  // namespace rover_led

#endif  // ROVER_LED_LED_COMPONENTS_LED_PANEL_HPP_
