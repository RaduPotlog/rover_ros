#ifndef ROVER_LED_LED_COMPONENTS_SEGMENT_LAYER_HPP_
#define ROVER_LED_LED_COMPONENTS_SEGMENT_LAYER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "rover_led/led_components/segment_layer_interface.hpp"

namespace rover_led
{

class SegmentLayer : public SegmentLayerInterface
{

public:

    SegmentLayer(
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

    bool repeating_ = false;
};

}  // namespace rover_led

#endif  // ROVER_LED_LED_COMPONENTS_SEGMENT_LAYER_HPP_
