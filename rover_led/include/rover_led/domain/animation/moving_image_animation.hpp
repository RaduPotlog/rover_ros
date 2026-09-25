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

#ifndef ROVER_LED_DOMAIN_ANIMATION_MOVING_IMAGE_ANIMATION_HPP_
#define ROVER_LED_DOMAIN_ANIMATION_MOVING_IMAGE_ANIMATION_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "boost/gil.hpp"
#include "boost/gil/extension/toolbox/color_spaces/gray_alpha.hpp"

#include "rover_led/domain/animation/image_animation.hpp"

namespace gil = boost::gil;

namespace rover_led
{

class MovingImageAnimation : public ImageAnimation
{

public:
  
    explicit MovingImageAnimation(std::shared_ptr<const IImageSource> image_source)
    : ImageAnimation(std::move(image_source)) {}
    ~MovingImageAnimation() {}

    void initialize(
        const YAML::Node & animation_description, 
        const std::size_t num_led,
        const float controller_frequency) override;

    void setParam(const std::string & param) override;

protected:

    std::vector<std::uint8_t> updateFrame() override;

private:
  
    float image_position_ = 0.0f;
    float default_image_position_ = 0.0f;
    bool default_image_position_set_ = false;
    bool image_mirrored_ = false;
    bool position_mirrored_ = false;
    std::size_t image_center_offset_ = 0;
    std::size_t image_object_width_ = 0;
    std::int32_t image_start_offset_ = 0;
    std::size_t splash_duration_ = 0;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_ANIMATION_MOVING_IMAGE_ANIMATION_HPP_