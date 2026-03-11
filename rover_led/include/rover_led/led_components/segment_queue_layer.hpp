#ifndef ROVER_LED_LED_COMPONENTS_SEGMENT_QUEUE_LAYER_HPP_
#define ROVER_LED_LED_COMPONENTS_SEGMENT_QUEUE_LAYER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "pluginlib/class_loader.hpp"

#include "rover_led/animation/animation.hpp"
#include "rover_led/led_components/segment_layer_interface.hpp"

namespace rover_led
{

class SegmentQueueLayer : public SegmentLayerInterface
{

public:
  
    SegmentQueueLayer(
        const std::size_t num_led, 
        const bool invert_led_order, 
        const float controller_frequency);

    void setAnimation(
        const std::string & type, 
        const YAML::Node & animation_description, 
        const bool repeating,
        const std::string & param = "") override;

    void updateAnimation() override;

protected:
  
    std::deque<std::shared_ptr<rover_led::Animation>> animations_queue_;
    std::size_t max_queue_size_ = 10;
};

}  // namespace rover_led

#endif  // ROVER_LED_LED_COMPONENTS_SEGMENT_QUEUE_LAYER_HPP_
