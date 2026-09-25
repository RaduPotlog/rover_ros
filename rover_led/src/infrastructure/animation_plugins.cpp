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

// pluginlib registration of the domain animations (see plugins.xml).

#include <memory>

#include "pluginlib/class_list_macros.hpp"

#include "rover_led/domain/animation/animation.hpp"
#include "rover_led/domain/animation/image_animation.hpp"
#include "rover_led/domain/animation/moving_image_animation.hpp"
#include "rover_led/infrastructure/png_image_file_source.hpp"

// pluginlib builds plugins with their default constructor, so each image animation is registered
// through a subclass that passes it the PNG file reader. plugins.xml keeps the domain class names
// as the lookup names: they are API, animation catalogs reference them.
namespace rover_led
{

class ImageAnimationPlugin : public ImageAnimation
{

public:

    ImageAnimationPlugin() : ImageAnimation(std::make_shared<PngImageFileSource>()) {}
};

class MovingImageAnimationPlugin : public MovingImageAnimation
{

public:

    MovingImageAnimationPlugin() : MovingImageAnimation(std::make_shared<PngImageFileSource>()) {}
};

}  // namespace rover_led

PLUGINLIB_EXPORT_CLASS(rover_led::ImageAnimationPlugin, rover_led::Animation)
PLUGINLIB_EXPORT_CLASS(rover_led::MovingImageAnimationPlugin, rover_led::Animation)
