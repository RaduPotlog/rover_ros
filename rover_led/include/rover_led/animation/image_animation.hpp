#ifndef ROVER_LED_IMAGE_ANIMATION_HPP_
#define ROVER_LED_IMAGE_ANIMATION_HPP_

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "boost/gil.hpp"
#include "boost/gil/extension/toolbox/color_spaces/gray_alpha.hpp"

#include "rover_led/animation/animation.hpp"

namespace gil = boost::gil;

namespace rover_led
{

class ImageAnimation : public Animation
{

public:
   
    ImageAnimation() {}
    
    ~ImageAnimation() {}

    void initialize(
        const YAML::Node & animation_description, 
        const std::size_t num_led,
        const float controller_frequency) override;

protected:

    std::vector<std::uint8_t> updateFrame() override;

    std::filesystem::path parseImagePath(const std::string & image_path) const;

    gil::rgba8_image_t rgbaImageResize(
        const gil::rgba8_image_t & image, 
        const std::size_t width, 
        const std::size_t height) const;

    void rgbaImageConvertColor(
        gil::rgba8_image_t & image, 
        const std::uint32_t color) const;

    gil::gray_alpha8_image_t rgbaImageConvertToGrey(
        const gil::rgba8_image_t & image) const;

    void greyImageNormalizeBrightness(gil::gray_alpha8_image_t & image) const;

protected:

    gil::rgba8_image_t image_;
};

}  // namespace rover_led

#endif  // ROVER_LED_IMAGE_ANIMATION_HPP_