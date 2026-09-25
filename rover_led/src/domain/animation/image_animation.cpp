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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "rover_led/domain/animation/image_animation.hpp"

#include "yaml-cpp/yaml.h"
#include "boost/gil.hpp"
#include "boost/gil/extension/numeric/resample.hpp"
#include "boost/gil/extension/numeric/sampler.hpp"

#include "rover_utils/yaml_utils.hpp"

namespace rover_led
{

void ImageAnimation::initialize(
    const YAML::Node & animation_description, 
    const std::size_t num_led,
    const float controller_frequency)
{
    Animation::initialize(animation_description, num_led, controller_frequency);

    image_ = rgbaImageResize(
        readImage(animation_description), this->getNumberOfLeds(), this->getAnimationLength());

    applyColorOption(animation_description);
}

gil::rgba8_image_t ImageAnimation::readImage(const YAML::Node & animation_description) const
{
    const auto name = rover_utils::getYAMLKeyValue<std::string>(animation_description, "image");
    const auto image = image_source_->read(name);

    if (image.width == 0 || image.height == 0 ||
        image.pixels.size() != image.width * image.height * kRGBAColorLen) {
        throw std::runtime_error(
            "Image '" + name + "' is not a valid RGBA image: " + std::to_string(image.width) + "x" +
            std::to_string(image.height) + " with " + std::to_string(image.pixels.size()) + " bytes");
    }

    gil::rgba8_image_t base_image(image.width, image.height);
    auto base_view = gil::view(base_image);

    for (std::size_t y = 0; y < image.height; y++) {
        for (std::size_t x = 0; x < image.width; x++) {
            const auto * p = &image.pixels[(y * image.width + x) * kRGBAColorLen];
            base_view(x, y) = gil::rgba8_pixel_t(p[0], p[1], p[2], p[3]);
        }
    }

    return base_image;
}

void ImageAnimation::applyColorOption(const YAML::Node & animation_description)
{
    if (animation_description["color"]) {
        rgbaImageConvertColor(image_, animation_description["color"].as<std::uint32_t>());
    }
}

std::vector<std::uint8_t> ImageAnimation::updateFrame()
{
    std::vector<std::uint8_t> frame;
  
    for (std::size_t i = 0; i < this->getNumberOfLeds(); i++) {
        auto pixel = gil::const_view(image_)(i, this->getAnimationIteration());
        frame.push_back(pixel[0]);
        frame.push_back(pixel[1]);
        frame.push_back(pixel[2]);
        frame.push_back(pixel[3]);
    }

    return frame;
}

gil::rgba8_image_t ImageAnimation::rgbaImageResize(
    const gil::rgba8_image_t & image, 
    const std::size_t width, 
    const std::size_t height) const
{
    gil::rgba8_image_t resized_image(width, height);
    gil::resize_view(gil::const_view(image), view(resized_image), gil::bilinear_sampler());

    return resized_image;
}

void ImageAnimation::rgbaImageConvertColor(
    gil::rgba8_image_t & image, 
    const std::uint32_t color) const
{
    auto grey_image = rgbaImageConvertToGrey(image);
    greyImageNormalizeBrightness(grey_image);

    auto r = (std::uint32_t(color) >> 16) & (0xFF);
    auto g = (std::uint32_t(color) >> 8) & (0xFF);
    auto b = (std::uint32_t(color)) & (0xFF);

    gil::transform_pixels(
        gil::const_view(grey_image), gil::view(image),
        [r, g, b](const gil::gray_alpha8_pixel_t & pixel) {
            return gil::rgba8_pixel_t(
                static_cast<std::uint8_t>(pixel[0] * r / 255),
                static_cast<std::uint8_t>(pixel[0] * g / 255),
                static_cast<std::uint8_t>(pixel[0] * b / 255), pixel[1]);
    });
}

gil::gray_alpha8_image_t ImageAnimation::rgbaImageConvertToGrey(
    const gil::rgba8_image_t & image) const
{
    gil::gray_alpha8_image_t grey_image(image.dimensions());
  
    gil::transform_pixels(
        gil::const_view(image), gil::view(grey_image), [](const gil::rgba8_pixel_t & pixel) {
            return gil::gray_alpha8_pixel_t(
                static_cast<std::uint8_t>(0.299 * pixel[0] + 0.587 * pixel[1] + 0.114 * pixel[2]),
                pixel[3]);
    });
  
    return grey_image;
}

void ImageAnimation::greyImageNormalizeBrightness(gil::gray_alpha8_image_t & image) const
{
    std::uint8_t max_value = *std::max_element(
        gil::nth_channel_view(gil::const_view(image), 0).begin(),
        gil::nth_channel_view(gil::const_view(image), 0).end());
    
    gil::transform_pixels(
    
    gil::const_view(image), gil::view(image), [max_value](const gil::gray_alpha8_pixel_t & pixel) {
        return gil::gray_alpha8_pixel_t(
            static_cast<std::uint8_t>(float(pixel[0]) / float(max_value) * 255), pixel[1]);
    });
}

}  // namespace rover_led