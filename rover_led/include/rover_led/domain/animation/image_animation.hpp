// Copyright 2025 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROVER_LED_DOMAIN_ANIMATION_IMAGE_ANIMATION_HPP_
#define ROVER_LED_DOMAIN_ANIMATION_IMAGE_ANIMATION_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "boost/gil.hpp"
#include "boost/gil/extension/toolbox/color_spaces/gray_alpha.hpp"

#include "rover_led/domain/animation/animation.hpp"
#include "rover_led/domain/ports/image_source.hpp"

namespace gil = boost::gil;

namespace rover_led
{

class ImageAnimation : public Animation
{

public:
   
    // Gets its image from image_source (non-null), by the description's "image" value.
    explicit ImageAnimation(std::shared_ptr<const IImageSource> image_source)
    : image_source_(std::move(image_source)) {}
    
    ~ImageAnimation() {}

    void initialize(
        const YAML::Node & animation_description, 
        const std::size_t num_led,
        const float controller_frequency) override;

protected:

    std::vector<std::uint8_t> updateFrame() override;

    // Reads the description's "image" through the image source, unscaled. Throws
    // std::runtime_error if the source can't provide it, or if the image is empty or its pixels
    // don't match its size.
    gil::rgba8_image_t readImage(const YAML::Node & animation_description) const;

    // Recolours image_ with the description's optional "color" (0xRRGGBB); no-op without it.
    // Call once image_ has its final size: recolouring normalizes to its brightest pixel.
    void applyColorOption(const YAML::Node & animation_description);

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

private:

    std::shared_ptr<const IImageSource> image_source_;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_ANIMATION_IMAGE_ANIMATION_HPP_