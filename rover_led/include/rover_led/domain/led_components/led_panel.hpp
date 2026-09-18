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

#ifndef ROVER_LED_DOMAIN_LED_COMPONENTS_LED_PANEL_HPP_
#define ROVER_LED_DOMAIN_LED_COMPONENTS_LED_PANEL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rover_led
{

// Segments write into a logical frame. With rows > 1 the physical strip is
// folded into that many serpentine rows: logical LED q is column q / rows
// (counted from the end LED 0 is on), row q % rows, and every odd row runs
// back the other way. getFrame() returns the frame in physical (wire) order.
class LedPanel
{

public:
    
    LedPanel(const std::size_t num_led, const std::size_t rows = 1);

    ~LedPanel() = default;

    void updateFrame(const std::size_t iterator_first, const std::vector<std::uint8_t> & values);

    std::vector<std::uint8_t> getFrame() const;
    
    std::size_t getNumberOfLeds() const 
    { 
        return num_led_; 
    }

    std::size_t getRows() const
    {
        return rows_;
    }

private:

    std::size_t physicalIndex(const std::size_t logical_index) const;

    const std::size_t num_led_;
    const std::size_t rows_;
    std::vector<std::uint8_t> frame_;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_LED_COMPONENTS_LED_PANEL_HPP_
