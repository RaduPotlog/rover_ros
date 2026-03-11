#ifndef ROVER_LED_LED_COMPONENTS_SEGMENT_CONVERTER_HPP_
#define ROVER_LED_LED_COMPONENTS_SEGMENT_CONVERTER_HPP_

#include <memory>
#include <unordered_map>
#include <vector>

#include "rover_led/led_components/led_panel.hpp"
#include "rover_led/led_components/led_segment.hpp"

namespace rover_led
{

class SegmentConverter
{
public:
  
    SegmentConverter() = default;
  
    ~SegmentConverter() = default;

    void convert(
        const std::unordered_map<std::string, std::shared_ptr<LedSegment>> & segments,
        const std::unordered_map<std::size_t, std::shared_ptr<LedPanel>> & panels);
};

}  // namespace rover_led

#endif  // ROVER_LED_LED_COMPONENTS_SEGMENT_CONVERTER_HPP_
