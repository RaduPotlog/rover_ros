#ifndef ROVER_LED_MOVING_IMAGE_ANIMATION_HPP_
#define ROVER_LED_MOVING_IMAGE_ANIMATION_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "boost/gil.hpp"
#include "boost/gil/extension/toolbox/color_spaces/gray_alpha.hpp"

#include "rover_led/animation/image_animation.hpp"

namespace gil = boost::gil;

namespace rover_led
{

class MovingImageAnimation : public ImageAnimation
{

public:
  
    MovingImageAnimation() {}
    ~MovingImageAnimation() {}

    void initialize(
        const YAML::Node & animation_description, 
        const std::size_t num_led,
        const float controller_frequency);

protected:

    std::vector<std::uint8_t> updateFrame();

    void setParam(const std::string & param);

private:
  
    float image_position_;
    float default_image_position_;
    bool default_image_position_set_;
    bool image_mirrored_;
    bool position_mirrored_;
    std::size_t image_center_offset_;
    std::size_t image_object_width_;
    std::int32_t image_start_offset_;
    std::size_t splash_duration_;
};

}  // namespace rover_led

#endif  // ROVER_LED_MOVING_IMAGE_ANIMATION_HPP_