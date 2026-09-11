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

#ifndef ROVER_LED_APPLICATION_SET_BRIGHTNESS_USE_CASE_HPP_
#define ROVER_LED_APPLICATION_SET_BRIGHTNESS_USE_CASE_HPP_

#include <memory>
#include <vector>

#include "rover_led/domain/sk9822_frame_encoder.hpp"

namespace rover_led
{

// Sets the same global brightness on every LED channel.
class SetBrightnessUseCase
{

public:

    explicit SetBrightnessUseCase(std::vector<std::shared_ptr<SK9822FrameEncoder>> encoders);

    // brightness in [0.0, 1.0]; throws std::out_of_range otherwise, leaving
    // every channel unchanged.
    void execute(const float brightness);

private:

    std::vector<std::shared_ptr<SK9822FrameEncoder>> encoders_;
};

}  // namespace rover_led

#endif  // ROVER_LED_APPLICATION_SET_BRIGHTNESS_USE_CASE_HPP_
