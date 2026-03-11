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
