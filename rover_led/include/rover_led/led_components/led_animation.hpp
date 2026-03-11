#ifndef ROVER_LED_LED_COMPONENTS_LED_ANIMATION_HPP_
#define ROVER_LED_LED_COMPONENTS_LED_ANIMATION_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "rclcpp/time.hpp"

#include "rover_led/led_components/led_segment.hpp"

namespace rover_led
{

struct AnimationDescription
{
    std::string type;
    std::vector<std::string> segments;
    YAML::Node animation;
};

struct LedAnimationDescription
{
    std::uint8_t id;
    std::uint8_t priority;
    std::string name;
    float timeout;
    std::vector<AnimationDescription> animations;
};

class LedAnimation
{

public:
  
    LedAnimation(
        const LedAnimationDescription & led_animation_description,
        const std::unordered_map<std::string, std::shared_ptr<LedSegment>> & segments,
        const rclcpp::Time & init_time)
    : led_animation_description_(led_animation_description)
    , init_time_(init_time)
    , repeating_(false)
    , param_("")
    {
        for (const auto & animation : led_animation_description_.animations) {
            for (const auto & segment : animation.segments) {
                if (segments.find(segment) == segments.end()) {
                    throw std::runtime_error("No segment with name: " + segment + ".");
                }

                animation_segments_.push_back(segments.at(segment));
            }
        }
    }

    ~LedAnimation() 
    {

    }

    std::string getName() const 
    { 
        return led_animation_description_.name; 
    }
    
    std::uint8_t getPriority() const 
    { 
        return led_animation_description_.priority; 
    }
    
    std::vector<AnimationDescription> getAnimations() const
    {
        return led_animation_description_.animations;
    }

    rclcpp::Time getInitTime() const 
    {  
        return init_time_; 
    }

    float getTimeout() const 
    { 
        return led_animation_description_.timeout; 
    }

    bool isRepeating() const 
    { 
        return repeating_; 
    }
  
    std::string getParam() const 
    { 
        return param_; 
    }

    void setRepeating(const bool value) 
    { 
        repeating_ = value; 
    }
    
    void setParam(const std::string & param) 
    { 
        param_ = param; 
    }

    static constexpr std::uint8_t kDefaultPriority = 3;
    static constexpr float kDefaultTimeout = 120.0f;
    static constexpr std::array<std::uint8_t, 4> kValidPriorities = {0, 1, 2, 3};

private:
  
    const LedAnimationDescription led_animation_description_;
    rclcpp::Time init_time_;

    bool repeating_;
    std::string param_;
    std::vector<std::shared_ptr<LedSegment>> animation_segments_;
};

}  // namespace rover_led

#endif  // ROVER_LED_LED_COMPONENTS_LED_ANIMATION_HPP_
