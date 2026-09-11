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

#include "rover_led/application/set_brightness_use_case.hpp"

#include <stdexcept>
#include <utility>

namespace rover_led
{

SetBrightnessUseCase::SetBrightnessUseCase(std::vector<std::shared_ptr<SK9822FrameEncoder>> encoders)
: encoders_(std::move(encoders))
{

}

void SetBrightnessUseCase::execute(const float brightness)
{
    if (brightness < 0.0f || brightness > 1.0f) {
        throw std::out_of_range("Brightness out of range [0.0, 1.0].");
    }

    for (auto & encoder : encoders_) {
        encoder->setGlobalBrightness(brightness);
    }
}

}  // namespace rover_led
