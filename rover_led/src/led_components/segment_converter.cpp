#include "rover_led/led_components/segment_converter.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rover_led/led_components/led_panel.hpp"
#include "rover_led/led_components/led_segment.hpp"

namespace rover_led
{

void SegmentConverter::convert(
    const std::unordered_map<std::string, std::shared_ptr<LedSegment>> & segments,
    const std::unordered_map<std::size_t, std::shared_ptr<LedPanel>> & panels)
{
  
    for (const auto & [segment_name, segment] : segments) {
        if (!segment->hasAnimation()) {
            continue;
        }

        try {
            auto panel = panels.at(segment->getChannel());
            auto frame = segment->getAnimationFrame();
        
            for (std::size_t i = 3; i < frame.size(); i += 4) {
                frame[i] = 255;
            }

            panel->updateFrame(segment->getFirstLedPosition(), frame);
        } catch (const std::runtime_error & e) {
            throw std::runtime_error("Failed to convert '" + segment_name +
                "' segment animation to panel frame. Error: " + std::string(e.what()));
        }
    }
}

}  // namespace rover_led
