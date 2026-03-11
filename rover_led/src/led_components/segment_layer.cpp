#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "rover_led/led_components/segment_layer.hpp"

#include "yaml-cpp/yaml.h"
#include "rclcpp/logging.hpp"

#include "rover_led/animation/animation.hpp"
#include "rover_led/led_components/segment_layer_interface.hpp"
#include "rover_utils/yaml_utils.hpp"

namespace rover_led
{

SegmentLayer::SegmentLayer(
    const std::size_t num_led, 
    const bool invert_led_order, 
    const float controller_frequency)
: SegmentLayerInterface(num_led, invert_led_order, controller_frequency)
{

}

void SegmentLayer::setAnimation(
    const std::string & type, 
    const YAML::Node & animation_description, 
    const bool repeating,
    const std::string & param)
{
    std::shared_ptr<rover_led::Animation> animation;
    
    try {
        animation = animation_loader_->createSharedInstance(type);
    } catch (pluginlib::PluginlibException & e) {
        throw std::runtime_error("The plugin failed to load. Error: " + std::string(e.what()));
    }

    try {
        animation->initialize(animation_description, num_led_, controller_frequency_);
        animation->setParam(param);
    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Failed to initialize animation: " + std::string(e.what()));
    } catch (const std::out_of_range & e) {
        throw std::runtime_error("Failed to initialize animation: " + std::string(e.what()));
    }

    animation_ = std::move(animation);
    animation_finished_ = false;
    repeating_ = repeating;
}

void SegmentLayer::updateAnimation()
{
    if (!animation_) {
        throw std::runtime_error("Segment animation not defined.");
    }

    if (animation_->isFinished()) {
        animation_finished_ = true;
    }

    if (animation_finished_ && animation_ && repeating_) {
        animation_->reset();
        animation_finished_ = false;
    }

    try {
        animation_->update();
    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Failed to update animation: " + std::string(e.what()));
    }
}

}  // namespace rover_led
