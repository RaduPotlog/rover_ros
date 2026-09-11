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

// pluginlib registration of the domain animations (see plugins.xml). The
// plugin names are API: animation catalogs reference them.

#include "pluginlib/class_list_macros.hpp"

#include "rover_led/domain/animation/animation.hpp"
#include "rover_led/domain/animation/image_animation.hpp"
#include "rover_led/domain/animation/moving_image_animation.hpp"

PLUGINLIB_EXPORT_CLASS(rover_led::ImageAnimation, rover_led::Animation)
PLUGINLIB_EXPORT_CLASS(rover_led::MovingImageAnimation, rover_led::Animation)
