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

#include "rover_led/domain/animation/moving_image_animation.hpp"

#include "yaml-cpp/yaml.h"
#include "boost/gil.hpp"
#include "boost/gil/extension/numeric/resample.hpp"
#include "boost/gil/extension/numeric/sampler.hpp"

#include "rover_utils/yaml_utils.hpp"

namespace rover_led
{

namespace
{

// Optional description keys: a missing or unconvertible value falls back to
// the default (getYAMLKeyValue reports both as std::runtime_error).
template <typename T>
T getOptionalKeyValue(const YAML::Node & description, const std::string & key, const T default_value)
{
    try {
        return rover_utils::getYAMLKeyValue<T>(description, key);
    } catch (const std::runtime_error & /*e*/) {
        return default_value;
    }
}

}  // namespace

void MovingImageAnimation::initialize(
    const YAML::Node & animation_description, 
    const std::size_t num_led,
    const float controller_frequency)
{
    Animation::initialize(animation_description, num_led, controller_frequency);

    const auto base_image = readImage(animation_description);

    image_center_offset_ = getOptionalKeyValue<std::int16_t>(animation_description, "center_offset", 0);
    image_object_width_ = getOptionalKeyValue<std::int16_t>(animation_description, "object_width", 0);

    const float image_start_offset_time = std::clamp(
        getOptionalKeyValue<float>(animation_description, "start_offset", 0.0f), -20.0f, 20.0f);
    image_start_offset_ = int(round(image_start_offset_time * controller_frequency));

    // 0 means "use the image height", see below.
    const float splash_duration_time = std::clamp(
        getOptionalKeyValue<float>(animation_description, "splash_duration", 0.0f), 0.0f, 20.0f);
    splash_duration_ = int(round(splash_duration_time * controller_frequency));

    image_mirrored_ = getOptionalKeyValue<bool>(animation_description, "image_mirrored", false);
    position_mirrored_ = getOptionalKeyValue<bool>(animation_description, "position_mirrored", false);

    // Without a default position, setParam() requires a non-empty param.
    default_image_position_set_ = false;

    try {
        default_image_position_ = std::clamp(
            rover_utils::getYAMLKeyValue<float>(animation_description, "default_image_position"), 0.0f, 1.0f);
        default_image_position_set_ = true;
    } catch (const std::runtime_error & /*e*/) {
    }

    if (splash_duration_ > 0) {
        image_ = rgbaImageResize(base_image, base_image.width(), splash_duration_);
    } else {
        splash_duration_ = base_image.height();
        image_ = base_image;
    }

    applyColorOption(animation_description);
}

void MovingImageAnimation::setParam(const std::string & param)
{
    if (default_image_position_set_ && param.empty()) {
        image_position_ = default_image_position_;
    
        if (position_mirrored_) {
          image_position_ = 1.0f - image_position_;
        }

        return;
    }

    try {
        image_position_ = std::clamp(std::stof(param), 0.0f, 1.0f);
    
        if (position_mirrored_) {
            image_position_ = 1.0f - image_position_;
        }
    } catch (const std::invalid_argument & /*e*/) {
        throw std::runtime_error("Can not cast param to float!");
    }
}

std::vector<std::uint8_t> MovingImageAnimation::updateFrame()
{
    int16_t left_edge_position;
  
    if (image_mirrored_) {
        left_edge_position = static_cast<int>(image_position_ * static_cast<int>(getNumberOfLeds() - (image_object_width_))) -
            (image_.width() - image_center_offset_ - image_object_width_);
    } else {
        left_edge_position = static_cast<int>(image_position_ * static_cast<int>(getNumberOfLeds() - (image_object_width_))) -
            image_center_offset_;
    }
  
    int16_t right_edge_position = left_edge_position + (image_.width());
    int16_t top_edge_position = image_start_offset_;
    int16_t bottom_edge_position = top_edge_position + splash_duration_;

    size_t left_range = std::clamp(left_edge_position, static_cast<int16_t>(0), static_cast<int16_t>(getNumberOfLeds()));
    size_t right_range = std::clamp(right_edge_position, static_cast<int16_t>(0), static_cast<int16_t>(getNumberOfLeds()));
    size_t top_range = std::clamp(top_edge_position, static_cast<int16_t>(0), static_cast<int16_t>(getAnimationLength()));
    size_t bottom_range = std::clamp(bottom_edge_position, static_cast<int16_t>(0), static_cast<int16_t>(getAnimationLength()));

    std::vector<std::uint8_t> frame;
  
    for (std::size_t i = 0; i < getNumberOfLeds(); i++) {
        if (i >= left_range && i < right_range && 
            getAnimationIteration() >= top_range &&
            getAnimationIteration() < bottom_range) {
            
            size_t pixel_index;
            
            if (image_mirrored_) {
                pixel_index = image_.width() - (i - left_edge_position) - 1;
            } else {
                pixel_index = i - left_edge_position;
            }

            auto pixel = gil::const_view(image_)(
            pixel_index, getAnimationIteration() - top_edge_position);
            frame.push_back(pixel[0]);
            frame.push_back(pixel[1]);
            frame.push_back(pixel[2]);
            frame.push_back(pixel[3]);
        } else {
            frame.push_back(0);
            frame.push_back(0);
            frame.push_back(0);
            frame.push_back(0);
        }
    }

    return frame;
}

}  // namespace rover_led