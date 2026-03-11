#ifndef ROVER_LED_LED_COMPONENTS_SEGMENT_LAYER_INTERFACE_HPP_
#define ROVER_LED_LED_COMPONENTS_SEGMENT_LAYER_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "pluginlib/class_loader.hpp"

#include "rover_led/animation/animation.hpp"

namespace rover_led
{

class SegmentLayerInterface
{

public:

    SegmentLayerInterface(
        const std::size_t num_led, 
        const bool invert_led_order, 
        const float controller_frequency)
    : controller_frequency_(controller_frequency)
    , invert_led_order_(invert_led_order)
    , num_led_(num_led)
    {
        animation_loader_ = std::make_shared<pluginlib::ClassLoader<rover_led::Animation>>(
        "rover_led", "rover_led::Animation");
    }

    virtual ~SegmentLayerInterface()
    {
        animation_.reset();
        animation_loader_.reset();
    }

    virtual void setAnimation(
        const std::string & type, 
        const YAML::Node & animation_description, 
        const bool repeating,
        const std::string & param = "") = 0;

    virtual void updateAnimation() = 0;

    bool isAnimationFinished() const { return animation_finished_; }

    std::vector<std::uint8_t> getAnimationFrame() const
    {
        if (animation_finished_ || !animation_) {
            return std::vector<std::uint8_t>(4 * num_led_, 0);
        }
  
        return animation_->getFrame(invert_led_order_);
    }
    
    float getAnimationProgress() const
    {
        if (!animation_) {
            throw std::runtime_error("Segment animation not defined.");
        }

        return animation_->getProgress();
    }

    void resetAnimation()
    {
        if (!animation_) {
            throw std::runtime_error("Segment animation not defined.");
        }

        animation_->reset();
        animation_finished_ = false;
    }

    bool hasAnimation() const 
    { 
        return static_cast<bool>(animation_); 
    }

protected:
  
    std::shared_ptr<rover_led::Animation> animation_;

    const float controller_frequency_;
    bool invert_led_order_ = false;
    bool animation_finished_ = true;
    std::size_t num_led_;

    std::shared_ptr<pluginlib::ClassLoader<rover_led::Animation>> animation_loader_;
};

}  // namespace rover_led

#endif  // ROVER_LED_LED_COMPONENTS_SEGMENT_LAYER_INTERFACE_HPP_
