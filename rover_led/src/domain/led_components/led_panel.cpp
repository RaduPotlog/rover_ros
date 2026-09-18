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

#include "rover_led/domain/led_components/led_panel.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace rover_led
{

LedPanel::LedPanel(const std::size_t num_led, const std::size_t rows) : num_led_(num_led), rows_(rows)
{
    if (rows_ == 0 || num_led_ % rows_ != 0) {
        throw std::runtime_error(
            "Can not fold " + std::to_string(num_led_) + " LEDs into " + std::to_string(rows_) + " rows.");
    }

    frame_ = std::vector<std::uint8_t>(num_led_ * 4, 0);
}

std::vector<std::uint8_t> LedPanel::getFrame() const
{
    if (rows_ == 1) {
        return frame_;
    }

    std::vector<std::uint8_t> physical_frame(frame_.size());

    for (std::size_t i = 0; i < num_led_; i++) {
        std::copy_n(frame_.begin() + i * 4, 4, physical_frame.begin() + physicalIndex(i) * 4);
    }

    return physical_frame;
}

std::size_t LedPanel::physicalIndex(const std::size_t logical_index) const
{
    const std::size_t columns = num_led_ / rows_;
    const std::size_t column = logical_index / rows_;
    const std::size_t row = logical_index % rows_;

    return row * columns + (row % 2 ? columns - 1 - column : column);
}

void LedPanel::updateFrame(
    const std::size_t iterator_first, 
    const std::vector<std::uint8_t> & values)
{
    if (values.empty()) {
        throw std::runtime_error("The input values vector is empty.");
    }
    
    if (values.size() > frame_.size()) {
        throw std::runtime_error(
            "The size of the input values (" + std::to_string(values.size()) +
            ") exceeds the size of the frame (" + std::to_string(frame_.size()) + ").");
    }

    if (values.size() + iterator_first > frame_.size()) {
        throw std::runtime_error(
            "The input values vector can't fit into the frame at the given "
            "position (" +
            std::to_string(iterator_first) + "). The size of the values vector is " +
            std::to_string(values.size()) + ", but the remaining space in the frame is " +
            std::to_string(frame_.size() - iterator_first) + ".");
    }

    std::copy(values.begin(), values.end(), frame_.begin() + iterator_first);
}

}  // namespace rover_led
