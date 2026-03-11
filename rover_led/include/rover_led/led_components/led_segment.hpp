#ifndef ROVER_LED_LED_COMPONENTS_LED_SEGMENT_HPP_
#define ROVER_LED_LED_COMPONENTS_LED_SEGMENT_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/time.hpp"

#include "rover_led/animation/animation.hpp"
#include "rover_led/led_components/segment_layer_interface.hpp"

namespace rover_led
{

enum AnimationPriority {
    ERROR = 0,
    ALERT,
    INFO,
    STATE,
};

class LedSegment
{

public:
  
    LedSegment(
        const YAML::Node & segment_description, 
        const float controller_frequency);

    ~LedSegment() {};

  
    void setAnimation(
        const std::string & type, 
        const YAML::Node & animation_description, 
        const bool repeating,
        const std::uint8_t priority, 
        const std::string & param = "");

    void updateAnimation();

    bool isAnimationFinished(AnimationPriority layer) const;

    std::vector<std::uint8_t> getAnimationFrame() const;

    float getAnimationProgress(AnimationPriority layer) const;

    void resetAnimation(AnimationPriority layer);

    std::size_t getFirstLedPosition() const;

    std::size_t getChannel() const 
    { 
        return channel_; 
    }

    bool layerHasAnimation(AnimationPriority layer) const;

    bool hasAnimation() const;

protected:
  
    std::vector<std::uint8_t> mergeLayersFrames() const;
    
    void mergeFrames(
        std::vector<std::uint8_t> & base_frame, 
        const std::vector<std::uint8_t> & overlay_frame) const;

private:

    const float controller_frequency_;
    bool invert_led_order_ = false;
    std::size_t channel_;
    std::size_t first_led_iterator_;
    std::size_t last_led_iterator_;
    std::size_t num_led_;
    std::map<AnimationPriority, std::unique_ptr<SegmentLayerInterface>> layers_;
};

}  // namespace rover_led

#endif  // ROVER_LED_LED_COMPONENTS_LED_SEGMENT_HPP_
