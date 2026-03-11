#include "rover_led/led_components/led_panel.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace rover_led
{

LedPanel::LedPanel(const std::size_t num_led) : num_led_(num_led)
{
    frame_ = std::vector<std::uint8_t>(num_led_ * 4, 0);
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
